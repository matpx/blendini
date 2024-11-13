#pragma once

#include <cstdint>

// User-callback for querying opacity for a triangle (e.g. via a texture map),
// or for just ignoring specific triangles entirely.
// Given U/V barycentric co-ordinates on a triangle, should return the
// opacity (0-1) at that location. Just return 0 if you want to completely
// ignore an intersection.
typedef float RjmRayFilterFn(int32_t triIdx, int32_t rayIdx, float t, float u, float v, void *userdata);

struct RjmRayNode;
struct RjmRayLeaf;

// Tree structure containing your scene.
struct RjmRayTree {
  // Fill these in yourself:
  int32_t triCount;
  float *vtxs;    // one vec3 per vertex
  int32_t *tris;  // three vertex indices per triangle

  // These are built by the library:
  int32_t firstLeaf;
  int32_t *leafTris;
  RjmRayNode *nodes;
  RjmRayLeaf *leafs;
};

// Ray structure. Initialize this yourself.
struct RjmRay {
  float org[3], dir[3];  // input: origin+dir (doesn't need to be normalized)
  float t;               // input: maximum T to traverse, output: T of terminating intersection (if any)
  int32_t hit;           // output: triangle we hit (-1 if none)
  float u, v;            // output: barycentric coordinates of intersection
  float visibility;      // output: ratio of how much the ray was blocked by geometry
  float normal[3];       // output: normal at hit point
};

// Initializes a tree from its scene description.
// Do this before tracing any rays.
void rjm_buildraytree(RjmRayTree *tree);

// Frees the internal data for a tree.
void rjm_freeraytree(RjmRayTree *tree);

constexpr int32_t RJM_RAYTRACE_FIRSTHIT = -1;

// Traces a batch of rays against the tree.
// tree:            Your scene (call rjm_buildraytree on it first)
// nrays/rays:      Batch of rays to trace. No limit to how many.
// filter/userdata: Custom callback for filtering triangles (can be NULL)
//
// cutoff:
//    Set to RJM_RAYTRACE_FIRSTHIT to find the earliest intersection
//    along each ray (i.e. the one with the lowest 't' value).
//    Otherwise, this specifies a visibility cutoff (0-1), and the trace will
//    stop once the visibility falls below or equal to this value.
int32_t rjm_raytrace(const RjmRayTree *tree, const int32_t nrays, RjmRay *rays, const float cutoff,
                     RjmRayFilterFn *filter, void *userdata);
