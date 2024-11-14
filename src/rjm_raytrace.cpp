#include "rjm_raytrace.hpp"

#include <simde-amalgamated/x86/sse2.h>

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
  simde__m128 vecmin = simde_mm_set_ps1(std::numeric_limits<float>::max());
  simde__m128 vecmax = simde_mm_set_ps1(std::numeric_limits<float>::min());
  for (int32_t n = 0; n < triCount; n++) {
    int32_t *idx = tree->tris + tree->leafTris[triIndex + n] * 3;
    for (int32_t v = 0; v < 3; v++) {
      float *vtx = tree->vtxs + idx[v] * 3;
      simde__m128 pos = simde_mm_set_ps(vtx[0], vtx[2], vtx[1], vtx[0]);
      vecmin = simde_mm_min_ps(vecmin, pos);
      vecmax = simde_mm_max_ps(vecmax, pos);
    }
  }

  // Store off final bounds.
  float bmin[4], bmax[4], bdim[4];
  simde__m128 vecdim = simde_mm_sub_ps(vecmax, vecmin);
  simde_mm_storeu_ps(bmin, vecmin);
  simde_mm_storeu_ps(bmax, vecmax);
  simde_mm_storeu_ps(bdim, vecdim);
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
          simde__m128 e01x = simde_mm_set1_ps(v1[0] - v0[0]);
          simde__m128 e01y = simde_mm_set1_ps(v1[1] - v0[1]);
          simde__m128 e01z = simde_mm_set1_ps(v1[2] - v0[2]);
          simde__m128 e02x = simde_mm_set1_ps(v2[0] - v0[0]);
          simde__m128 e02y = simde_mm_set1_ps(v2[1] - v0[1]);
          simde__m128 e02z = simde_mm_set1_ps(v2[2] - v0[2]);

          // normal = cross(e01, e02)
          simde__m128 normalx = simde_mm_sub_ps(simde_mm_mul_ps(e01y, e02z), simde_mm_mul_ps(e01z, e02y));
          simde__m128 normaly = simde_mm_sub_ps(simde_mm_mul_ps(e01z, e02x), simde_mm_mul_ps(e01x, e02z));
          simde__m128 normalz = simde_mm_sub_ps(simde_mm_mul_ps(e01x, e02y), simde_mm_mul_ps(e01y, e02x));

          // Ray-triangle intersection.
          simde__m128 mask = simde_mm_setzero_ps();
          for (int32_t n = 0; n < nvec; n++) {
            int32_t p = n * 4;

            // pvec = cross(dir, e02)
            simde__m128 pvecx = simde_mm_sub_ps(simde_mm_mul_ps(simde_mm_load_ps(dy + p), e02z), simde_mm_mul_ps(simde_mm_load_ps(dz + p), e02y));
            simde__m128 pvecy = simde_mm_sub_ps(simde_mm_mul_ps(simde_mm_load_ps(dz + p), e02x), simde_mm_mul_ps(simde_mm_load_ps(dx + p), e02z));
            simde__m128 pvecz = simde_mm_sub_ps(simde_mm_mul_ps(simde_mm_load_ps(dx + p), e02y), simde_mm_mul_ps(simde_mm_load_ps(dy + p), e02x));

            // det = dot(e01, pvec)
            simde__m128 det =
                simde_mm_add_ps(simde_mm_mul_ps(e01x, pvecx), simde_mm_add_ps(simde_mm_mul_ps(e01y, pvecy), simde_mm_mul_ps(e01z, pvecz)));

            // tvec = org - vtx0
            simde__m128 tvecx = simde_mm_sub_ps(simde_mm_load_ps(rx + p), simde_mm_set_ps1(v0[0]));
            simde__m128 tvecy = simde_mm_sub_ps(simde_mm_load_ps(ry + p), simde_mm_set_ps1(v0[1]));
            simde__m128 tvecz = simde_mm_sub_ps(simde_mm_load_ps(rz + p), simde_mm_set_ps1(v0[2]));

            // qvec = cross(tvec, e01)
            simde__m128 qvecx = simde_mm_sub_ps(simde_mm_mul_ps(tvecy, e01z), simde_mm_mul_ps(tvecz, e01y));
            simde__m128 qvecy = simde_mm_sub_ps(simde_mm_mul_ps(tvecz, e01x), simde_mm_mul_ps(tvecx, e01z));
            simde__m128 qvecz = simde_mm_sub_ps(simde_mm_mul_ps(tvecx, e01y), simde_mm_mul_ps(tvecy, e01x));

            // u = dot(tvec, pvec) * inv_det
            // v = dot(dir, qvec) * inv_det
            // t = dot(e02, qvec) * inv_det
            simde__m128 u =
                simde_mm_add_ps(simde_mm_mul_ps(tvecx, pvecx), simde_mm_add_ps(simde_mm_mul_ps(tvecy, pvecy), simde_mm_mul_ps(tvecz, pvecz)));
            simde__m128 v =
                simde_mm_add_ps(simde_mm_mul_ps(simde_mm_load_ps(dx + p), qvecx),
                           simde_mm_add_ps(simde_mm_mul_ps(simde_mm_load_ps(dy + p), qvecy), simde_mm_mul_ps(simde_mm_load_ps(dz + p), qvecz)));
            simde__m128 t =
                simde_mm_add_ps(simde_mm_mul_ps(e02x, qvecx), simde_mm_add_ps(simde_mm_mul_ps(e02y, qvecy), simde_mm_mul_ps(e02z, qvecz)));
            simde__m128 inv_det = simde_mm_div_ps(simde_mm_set_ps1(1.0f), det);
            u = simde_mm_mul_ps(u, inv_det);
            v = simde_mm_mul_ps(v, inv_det);
            t = simde_mm_mul_ps(t, inv_det);

            // Intersection if all of:
            // u>=0, u<=1, v>=0, u+v<=1, t>=0, t<=maxt
            simde__m128 zero = simde_mm_setzero_ps();
            simde__m128 one = simde_mm_set_ps1(1.0f);
            simde__m128 prev = simde_mm_load_ps(maxt + p);
            simde__m128 isect = simde_mm_cmpge_ps(u, zero);
            isect = simde_mm_and_ps(isect, simde_mm_cmple_ps(u, one));
            isect = simde_mm_and_ps(isect, simde_mm_cmpge_ps(v, zero));
            isect = simde_mm_and_ps(isect, simde_mm_cmple_ps(simde_mm_add_ps(u, v), one));
            isect = simde_mm_and_ps(isect, simde_mm_cmpge_ps(t, zero));
            isect = simde_mm_and_ps(isect, simde_mm_cmple_ps(t, prev));

            mask = simde_mm_or_ps(mask, isect);
            simde_mm_store_ps((float *)out_mask + p, isect);
            simde_mm_store_ps((float *)out_u + p, u);
            simde_mm_store_ps((float *)out_v + p, v);
            simde_mm_store_ps((float *)out_t + p, t);
          }

          // See which ones hit.
          if (simde_mm_movemask_ps(mask) != 0) {
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
                      ray->normal[0] = simde_mm_cvtss_f32(normalx);
                      ray->normal[1] = simde_mm_cvtss_f32(normaly);
                      ray->normal[2] = simde_mm_cvtss_f32(normalz);

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
        simde__m128 bminx = simde_mm_set_ps1(node->bmin[0]);
        simde__m128 bminy = simde_mm_set_ps1(node->bmin[1]);
        simde__m128 bminz = simde_mm_set_ps1(node->bmin[2]);
        simde__m128 bmaxx = simde_mm_set_ps1(node->bmax[0]);
        simde__m128 bmaxy = simde_mm_set_ps1(node->bmax[1]);
        simde__m128 bmaxz = simde_mm_set_ps1(node->bmax[2]);

        simde__m128 mask = simde_mm_setzero_ps();

        // Ray-box slab test.
        for (int32_t n = 0; n < nvec; n++) {
          int32_t p = n * 4;
          // d0 = (bmin - org) * invdir
          // d1 = (bmax - org) * invdir
          simde__m128 d0x = simde_mm_mul_ps(simde_mm_sub_ps(bminx, simde_mm_load_ps(rx + p)), simde_mm_load_ps(ix + p));
          simde__m128 d0y = simde_mm_mul_ps(simde_mm_sub_ps(bminy, simde_mm_load_ps(ry + p)), simde_mm_load_ps(iy + p));
          simde__m128 d0z = simde_mm_mul_ps(simde_mm_sub_ps(bminz, simde_mm_load_ps(rz + p)), simde_mm_load_ps(iz + p));
          simde__m128 d1x = simde_mm_mul_ps(simde_mm_sub_ps(bmaxx, simde_mm_load_ps(rx + p)), simde_mm_load_ps(ix + p));
          simde__m128 d1y = simde_mm_mul_ps(simde_mm_sub_ps(bmaxy, simde_mm_load_ps(ry + p)), simde_mm_load_ps(iy + p));
          simde__m128 d1z = simde_mm_mul_ps(simde_mm_sub_ps(bmaxz, simde_mm_load_ps(rz + p)), simde_mm_load_ps(iz + p));

          // v0 = min(d0, d1)
          // v1 = max(d0, d1)
          simde__m128 v0x = simde_mm_min_ps(d0x, d1x);
          simde__m128 v0y = simde_mm_min_ps(d0y, d1y);
          simde__m128 v0z = simde_mm_min_ps(d0z, d1z);
          simde__m128 v1x = simde_mm_max_ps(d0x, d1x);
          simde__m128 v1y = simde_mm_max_ps(d0y, d1y);
          simde__m128 v1z = simde_mm_max_ps(d0z, d1z);

          // tmin = hmax(v0)
          // tmax = hmin(v1)
          simde__m128 tmin = simde_mm_max_ps(v0x, simde_mm_max_ps(v0y, v0z));
          simde__m128 tmax = simde_mm_min_ps(v1x, simde_mm_min_ps(v1y, v1z));

          simde__m128 prevt = simde_mm_load_ps(maxt + p);

          // hit if: (tmax >= 0) && (tmax >= tmin) && (tmin <= maxt)
          simde__m128 isect = simde_mm_cmpge_ps(tmax, simde_mm_setzero_ps());
          isect = simde_mm_and_ps(isect, simde_mm_cmpge_ps(tmax, tmin));
          isect = simde_mm_and_ps(isect, simde_mm_cmple_ps(tmin, prevt));

          mask = simde_mm_or_ps(mask, isect);  // accumulate results
          simde_mm_store_ps((float *)out_mask + p, isect);
        }

        // Check if any rays hit the box.
        if (simde_mm_movemask_ps(mask) != 0) {
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
