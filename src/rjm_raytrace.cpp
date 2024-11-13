#include "rjm_raytrace.hpp"

#include <xmmintrin.h>

#include <cassert>
#include <limits>
#include <utility>

// Tweak for maximum number of tris per leaf node.
constexpr int32_t RJM_MAX_RAYTREE_LEAF_TRIS = 4;

// Tweak for maximum rays to trace at once (limited by stack space, must be multiple of 4)
constexpr int32_t RJM_PACKET_SIZE = 128;

struct RjmRayNode {
  float bmin[3], bmax[3];
};

struct RjmRayLeaf {
  int32_t triIndex, triCount;
};

static int32_t *rjm_raytree_partition(RjmRayTree *tree, int32_t *left, int32_t *right, int32_t axis) {
  int32_t pivot = right[0];
  int32_t v0 = tree->tris[pivot * 3];
  float split = tree->vtxs[v0 * 3 + axis];
  int32_t *dest = left;
  for (int32_t *i = left; i < right; i++) {
    int32_t v0 = tree->tris[(*i) * 3];
    if (tree->vtxs[v0 * 3 + axis] < split) {
      std::swap(*dest, *i);
      dest++;
    }
  }
  std::swap(*dest, *right);
  return dest;
}

static void rjm_raytree_quickselect(RjmRayTree *tree, int32_t *left, int32_t *right, int32_t *mid, int32_t axis) {
  for (;;) {
    int32_t *pivot = rjm_raytree_partition(tree, left, right, axis);
    if (mid < pivot)
      right = pivot - 1;
    else if (mid > pivot)
      left = pivot + 1;
    else
      break;
  }
}

static void rjm_buildraynodes(RjmRayTree *tree, int32_t nodeIdx, int32_t triIndex, int32_t triCount) {
  if (nodeIdx >= tree->firstLeaf) {
    assert(triCount <= RJM_MAX_RAYTREE_LEAF_TRIS);
    RjmRayLeaf *leaf = tree->leafs + (nodeIdx - tree->firstLeaf);
    leaf->triIndex = triIndex;
    leaf->triCount = triCount;
    return;
  }

  // Simple object-median split algorithm. Performs reasonably
  // well, gives us a balanced, implicit tree, and is guaranteed
  // to always split.

  // Calculate bounds.
  __m128 vecmin = _mm_set_ps1(std::numeric_limits<float>::max());
  __m128 vecmax = _mm_set_ps1(std::numeric_limits<float>::min());
  for (int32_t n = 0; n < triCount; n++) {
    int32_t *idx = tree->tris + tree->leafTris[triIndex + n] * 3;
    for (int32_t v = 0; v < 3; v++) {
      float *vtx = tree->vtxs + idx[v] * 3;
      __m128 pos = _mm_set_ps(vtx[0], vtx[2], vtx[1], vtx[0]);
      vecmin = _mm_min_ps(vecmin, pos);
      vecmax = _mm_max_ps(vecmax, pos);
    }
  }

  // Store off final bounds.
  float bmin[4], bmax[4], bdim[4];
  __m128 vecdim = _mm_sub_ps(vecmax, vecmin);
  _mm_storeu_ps(bmin, vecmin);
  _mm_storeu_ps(bmax, vecmax);
  _mm_storeu_ps(bdim, vecdim);
  RjmRayNode *node = tree->nodes + nodeIdx;
  node->bmin[0] = bmin[0];
  node->bmin[1] = bmin[1];
  node->bmin[2] = bmin[2];
  node->bmax[0] = bmax[0];
  node->bmax[1] = bmax[1];
  node->bmax[2] = bmax[2];

  // Pick longest axis.
  int32_t axis = 0;
  if (bdim[1] > bdim[axis]) axis = 1;
  if (bdim[2] > bdim[axis]) axis = 2;

  // Partition.
  assert(triCount > 0);
  int32_t leftCount = triCount >> 1;
  int32_t *tris = tree->leafTris + triIndex;
  rjm_raytree_quickselect(tree, tris, tris + triCount - 1, tris + leftCount, axis);

  // Recurse.
  rjm_buildraynodes(tree, nodeIdx * 2 + 1, triIndex, leftCount);
  rjm_buildraynodes(tree, nodeIdx * 2 + 2, triIndex + leftCount, triCount - leftCount);
}

void rjm_buildraytree(RjmRayTree *tree) {
  // Pick how many nodes we want (must be a power of 2 for balanced trees)
  int32_t leafCount = 1;
  while (leafCount * RJM_MAX_RAYTREE_LEAF_TRIS < tree->triCount) leafCount <<= 1;

  // Allocate memory.
  tree->firstLeaf = leafCount - 1;
  tree->nodes = (RjmRayNode *)malloc(tree->firstLeaf * sizeof(RjmRayNode));
  tree->leafs = (RjmRayLeaf *)malloc(leafCount * sizeof(RjmRayLeaf));
  tree->leafTris = (int32_t *)malloc(tree->triCount * sizeof(int32_t));

  // Fill in initial leaf data.
  for (int32_t n = 0; n < tree->triCount; n++) tree->leafTris[n] = n;

  // Recursively partition.
  rjm_buildraynodes(tree, 0, 0, tree->triCount);
}

void rjm_freeraytree(RjmRayTree *tree) {
  free(tree->nodes);
  free(tree->leafs);
  free(tree->leafTris);
  tree->nodes = NULL;
  tree->leafs = NULL;
  tree->leafTris = NULL;
  tree->firstLeaf = -1;
}

int32_t rjm_raytrace(const RjmRayTree *tree, const int32_t nrays, RjmRay *rays, const float cutoff,
                     RjmRayFilterFn *filter, void *userdata) {
  // Allocate local SSE structures.
  alignas(16) float rx[RJM_PACKET_SIZE], ry[RJM_PACKET_SIZE], rz[RJM_PACKET_SIZE];
  alignas(16) float dx[RJM_PACKET_SIZE], dy[RJM_PACKET_SIZE], dz[RJM_PACKET_SIZE];
  alignas(16) float ix[RJM_PACKET_SIZE], iy[RJM_PACKET_SIZE], iz[RJM_PACKET_SIZE];
  alignas(16) float maxt[RJM_PACKET_SIZE];
  alignas(16) int32_t rayidx[RJM_PACKET_SIZE];

  alignas(16) int32_t out_mask[RJM_PACKET_SIZE];
  alignas(16) float out_u[RJM_PACKET_SIZE], out_v[RJM_PACKET_SIZE], out_t[RJM_PACKET_SIZE];

  int32_t stack[64], *top;
  int32_t hit_count = 0;

  // Process it in packets, in case they pass in a lot of rays at once.
  for (int32_t base = 0; base < nrays;) {
    int32_t npacket = nrays - base;
    if (npacket > RJM_PACKET_SIZE) npacket = RJM_PACKET_SIZE;
    int32_t next = base + npacket;
    RjmRay *raybatch = rays + base;

    // Copy rays into our local structure.
    for (int32_t n = 0; n < npacket; n++) {
      rx[n] = raybatch[n].org[0];
      ry[n] = raybatch[n].org[1];
      rz[n] = raybatch[n].org[2];
      dx[n] = raybatch[n].dir[0];
      dy[n] = raybatch[n].dir[1];
      dz[n] = raybatch[n].dir[2];
      ix[n] = 1.0f / raybatch[n].dir[0];  // relies on IEEE infinity
      iy[n] = 1.0f / raybatch[n].dir[1];
      iz[n] = 1.0f / raybatch[n].dir[2];
      maxt[n] = raybatch[n].t;

      raybatch[n].visibility = 1.0f;
      raybatch[n].hit = -1;
      raybatch[n].u = 0;
      raybatch[n].v = 0;
      rayidx[n] = base + n;
    }

    // Align up to multiple of 4.
    while (npacket & 3) {
      int32_t d = npacket, s = npacket - 1;
      rx[d] = rx[s];
      ry[d] = ry[s];
      rz[d] = rz[s];
      dx[d] = dx[s];
      dy[d] = dy[s];
      dz[d] = dz[s];
      ix[d] = ix[s];
      iy[d] = iy[s];
      iz[d] = iz[s];
      maxt[d] = maxt[s];
      rayidx[d] = -1;
      npacket++;
    }

    // Push terminator.
    top = stack;
    *top++ = 0;
    *top++ = 0;

    int32_t nodeIdx = 0;
    int32_t ncur = npacket;

    // Trace the tree.
    do {
      int32_t nvec = ncur >> 2;

      int32_t leafIdx = nodeIdx - tree->firstLeaf;
      if (leafIdx >= 0) {
        // Leaf, test each triangle.
        RjmRayLeaf *leaf = tree->leafs + leafIdx;
        int32_t *idxs = tree->leafTris + leaf->triIndex;
        int32_t triCount = leaf->triCount;
        while (triCount--) {
          // Read triangle data.
          int32_t triIdx = *idxs++;
          int32_t *tri = tree->tris + triIdx * 3;
          float *v0 = tree->vtxs + tri[0] * 3;
          float *v1 = tree->vtxs + tri[1] * 3;
          float *v2 = tree->vtxs + tri[2] * 3;

          // Edge vector.
          __m128 e01x = _mm_set1_ps(v1[0] - v0[0]);
          __m128 e01y = _mm_set1_ps(v1[1] - v0[1]);
          __m128 e01z = _mm_set1_ps(v1[2] - v0[2]);
          __m128 e02x = _mm_set1_ps(v2[0] - v0[0]);
          __m128 e02y = _mm_set1_ps(v2[1] - v0[1]);
          __m128 e02z = _mm_set1_ps(v2[2] - v0[2]);

          // Ray-triangle intersection.
          __m128 mask = _mm_setzero_ps();
          for (int32_t n = 0; n < nvec; n++) {
            int32_t p = n * 4;

            // pvec = cross(dir, e02)
            __m128 pvecx = _mm_sub_ps(_mm_mul_ps(_mm_load_ps(dy + p), e02z), _mm_mul_ps(_mm_load_ps(dz + p), e02y));
            __m128 pvecy = _mm_sub_ps(_mm_mul_ps(_mm_load_ps(dz + p), e02x), _mm_mul_ps(_mm_load_ps(dx + p), e02z));
            __m128 pvecz = _mm_sub_ps(_mm_mul_ps(_mm_load_ps(dx + p), e02y), _mm_mul_ps(_mm_load_ps(dy + p), e02x));

            // det = dot(e01, pvec)
            __m128 det =
                _mm_add_ps(_mm_mul_ps(e01x, pvecx), _mm_add_ps(_mm_mul_ps(e01y, pvecy), _mm_mul_ps(e01z, pvecz)));

            // tvec = org - vtx0
            __m128 tvecx = _mm_sub_ps(_mm_load_ps(rx + p), _mm_set_ps1(v0[0]));
            __m128 tvecy = _mm_sub_ps(_mm_load_ps(ry + p), _mm_set_ps1(v0[1]));
            __m128 tvecz = _mm_sub_ps(_mm_load_ps(rz + p), _mm_set_ps1(v0[2]));

            // qvec = cross(tvec, e01)
            __m128 qvecx = _mm_sub_ps(_mm_mul_ps(tvecy, e01z), _mm_mul_ps(tvecz, e01y));
            __m128 qvecy = _mm_sub_ps(_mm_mul_ps(tvecz, e01x), _mm_mul_ps(tvecx, e01z));
            __m128 qvecz = _mm_sub_ps(_mm_mul_ps(tvecx, e01y), _mm_mul_ps(tvecy, e01x));

            // u = dot(tvec, pvec) * inv_det
            // v = dot(dir, qvec) * inv_det
            // t = dot(e02, qvec) * inv_det
            __m128 u =
                _mm_add_ps(_mm_mul_ps(tvecx, pvecx), _mm_add_ps(_mm_mul_ps(tvecy, pvecy), _mm_mul_ps(tvecz, pvecz)));
            __m128 v =
                _mm_add_ps(_mm_mul_ps(_mm_load_ps(dx + p), qvecx),
                           _mm_add_ps(_mm_mul_ps(_mm_load_ps(dy + p), qvecy), _mm_mul_ps(_mm_load_ps(dz + p), qvecz)));
            __m128 t =
                _mm_add_ps(_mm_mul_ps(e02x, qvecx), _mm_add_ps(_mm_mul_ps(e02y, qvecy), _mm_mul_ps(e02z, qvecz)));
            __m128 inv_det = _mm_div_ps(_mm_set_ps1(1.0f), det);
            u = _mm_mul_ps(u, inv_det);
            v = _mm_mul_ps(v, inv_det);
            t = _mm_mul_ps(t, inv_det);

            // Intersection if all of:
            // u>=0, u<=1, v>=0, u+v<=1, t>=0, t<=maxt
            __m128 zero = _mm_setzero_ps();
            __m128 one = _mm_set_ps1(1.0f);
            __m128 prev = _mm_load_ps(maxt + p);
            __m128 isect = _mm_cmpge_ps(u, zero);
            isect = _mm_and_ps(isect, _mm_cmple_ps(u, one));
            isect = _mm_and_ps(isect, _mm_cmpge_ps(v, zero));
            isect = _mm_and_ps(isect, _mm_cmple_ps(_mm_add_ps(u, v), one));
            isect = _mm_and_ps(isect, _mm_cmpge_ps(t, zero));
            isect = _mm_and_ps(isect, _mm_cmple_ps(t, prev));

            mask = _mm_or_ps(mask, isect);
            _mm_store_ps((float *)out_mask + p, isect);
            _mm_store_ps((float *)out_u + p, u);
            _mm_store_ps((float *)out_v + p, v);
            _mm_store_ps((float *)out_t + p, t);
          }

          // See which ones hit.
          if (_mm_movemask_ps(mask) != 0) {
            for (int32_t n = 0; n < ncur; n++) {
              if (out_mask[n] < 0 && rayidx[n] >= 0) {
                RjmRay *ray = rays + rayidx[n];
                if (out_t[n] < ray->t) {
                  float opacity = 1.0f;
                  if (filter) opacity = filter(triIdx, rayidx[n], out_t[n], out_u[n], out_v[n], userdata);
                  if (cutoff >= 0) {
                    // Shadow mode, accumulate total visibility.
                    ray->visibility *= (1 - opacity);
                    if (ray->visibility <= cutoff) maxt[n] = 0;  // stop further testing
                  } else {
                    // Regular mode, find earliest intersection.
                    if (opacity >= 0.5f) {
                      ray->t = out_t[n];
                      ray->u = out_u[n];
                      ray->v = out_v[n];
                      ray->hit = triIdx;
                      ray->visibility = 0.0f;

                      ray->normal[0] =
                          _mm_cvtss_f32(e01y) * _mm_cvtss_f32(e02z) - _mm_cvtss_f32(e01z) * _mm_cvtss_f32(e02y);
                      ray->normal[1] =
                          _mm_cvtss_f32(e01z) * _mm_cvtss_f32(e02x) - _mm_cvtss_f32(e01x) * _mm_cvtss_f32(e02z);
                      ray->normal[2] =
                          _mm_cvtss_f32(e01x) * _mm_cvtss_f32(e02y) - _mm_cvtss_f32(e01y) * _mm_cvtss_f32(e02x);

                      hit_count++;

                      maxt[n] = out_t[n];
                    }
                  }
                }
              }
            }
          }
        }
      } else {
        // Node, test bounds.
        RjmRayNode *node = tree->nodes + nodeIdx;
        __m128 bminx = _mm_set_ps1(node->bmin[0]);
        __m128 bminy = _mm_set_ps1(node->bmin[1]);
        __m128 bminz = _mm_set_ps1(node->bmin[2]);
        __m128 bmaxx = _mm_set_ps1(node->bmax[0]);
        __m128 bmaxy = _mm_set_ps1(node->bmax[1]);
        __m128 bmaxz = _mm_set_ps1(node->bmax[2]);

        __m128 mask = _mm_setzero_ps();

        // Ray-box slab test.
        for (int32_t n = 0; n < nvec; n++) {
          int32_t p = n * 4;
          // d0 = (bmin - org) * invdir
          // d1 = (bmax - org) * invdir
          __m128 d0x = _mm_mul_ps(_mm_sub_ps(bminx, _mm_load_ps(rx + p)), _mm_load_ps(ix + p));
          __m128 d0y = _mm_mul_ps(_mm_sub_ps(bminy, _mm_load_ps(ry + p)), _mm_load_ps(iy + p));
          __m128 d0z = _mm_mul_ps(_mm_sub_ps(bminz, _mm_load_ps(rz + p)), _mm_load_ps(iz + p));
          __m128 d1x = _mm_mul_ps(_mm_sub_ps(bmaxx, _mm_load_ps(rx + p)), _mm_load_ps(ix + p));
          __m128 d1y = _mm_mul_ps(_mm_sub_ps(bmaxy, _mm_load_ps(ry + p)), _mm_load_ps(iy + p));
          __m128 d1z = _mm_mul_ps(_mm_sub_ps(bmaxz, _mm_load_ps(rz + p)), _mm_load_ps(iz + p));

          // v0 = min(d0, d1)
          // v1 = max(d0, d1)
          __m128 v0x = _mm_min_ps(d0x, d1x);
          __m128 v0y = _mm_min_ps(d0y, d1y);
          __m128 v0z = _mm_min_ps(d0z, d1z);
          __m128 v1x = _mm_max_ps(d0x, d1x);
          __m128 v1y = _mm_max_ps(d0y, d1y);
          __m128 v1z = _mm_max_ps(d0z, d1z);

          // tmin = hmax(v0)
          // tmax = hmin(v1)
          __m128 tmin = _mm_max_ps(v0x, _mm_max_ps(v0y, v0z));
          __m128 tmax = _mm_min_ps(v1x, _mm_min_ps(v1y, v1z));

          __m128 prevt = _mm_load_ps(maxt + p);

          // hit if: (tmax >= 0) && (tmax >= tmin) && (tmin <= maxt)
          __m128 isect = _mm_cmpge_ps(tmax, _mm_setzero_ps());
          isect = _mm_and_ps(isect, _mm_cmpge_ps(tmax, tmin));
          isect = _mm_and_ps(isect, _mm_cmple_ps(tmin, prevt));

          mask = _mm_or_ps(mask, isect);  // accumulate results
          _mm_store_ps((float *)out_mask + p, isect);
        }

        // Check if any rays hit the box.
        if (_mm_movemask_ps(mask) != 0) {
          // Re-order rays into ones that hit and ones that didn't.
          int32_t nhit = 0;
          while (nhit < ncur) {
            if (out_mask[nhit] >= 0) {
              // miss, move ray to the end
              int32_t d = nhit, s = --ncur;
              std::swap(rx[d], rx[s]);
              std::swap(ry[d], ry[s]);
              std::swap(rz[d], rz[s]);
              std::swap(dx[d], dx[s]);
              std::swap(dy[d], dy[s]);
              std::swap(dz[d], dz[s]);
              std::swap(ix[d], ix[s]);
              std::swap(iy[d], iy[s]);
              std::swap(iz[d], iz[s]);
              std::swap(maxt[d], maxt[s]);
              std::swap(rayidx[d], rayidx[s]);
              std::swap(out_mask[d], out_mask[s]);
            } else {
              // hit
              nhit++;
            }
          }

          if (ncur > 0) {
            ncur = (ncur + 3) & ~3;

            // Recurse in with only the rays that hit the node.
            *top++ = nodeIdx * 2 + 2;
            *top++ = ncur;
            nodeIdx = nodeIdx * 2 + 1;
            continue;
          }
        }
      }

      // Pull a new node off the stack.
      ncur = *--top;
      nodeIdx = *--top;
    } while (nodeIdx);

    base = next;
  }

  return hit_count;
}
