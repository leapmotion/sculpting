#pragma once
#include "cinder/CinderResources.h"

// shaders
#define RES_SKY_VERT_GLSL                     CINDER_RESOURCE( ../resources/, sky-vert.glsl, 116, GLSL )
#define RES_SKY_FRAG_GLSL                     CINDER_RESOURCE( ../resources/, sky-frag.glsl, 117, GLSL )
#define RES_PASSTHROUGH_VERT_GLSL             CINDER_RESOURCE( ../resources/, passthrough-vert.glsl, 118, GLSL )
#define RES_MATERIAL_VERT_GLSL                CINDER_RESOURCE( ../resources/, material-vert.glsl, 120, GLSL )
#define RES_MATERIAL_FRAG_GLSL                CINDER_RESOURCE( ../resources/, material-frag.glsl, 121, GLSL )
#define RES_BRUSH_VERT_GLSL                   CINDER_RESOURCE( ../resources/, brush-vert.glsl, 122, GLSL )
#define RES_METABALL_FRAG_GLSL                CINDER_RESOURCE( ../resources/, metaball-frag.glsl, 123, GLSL )
#define RES_SCREEN_FRAG_GLSL                  CINDER_RESOURCE( ../resources/, screen-frag.glsl, 131, GLSL )
#define RES_FXAA_FRAG_GLSL                    CINDER_RESOURCE( ../resources/, fxaa-frag.glsl, 133, GLSL )
#define RES_WIREFRAME_FRAG_GLSL               CINDER_RESOURCE( ../resources/, wireframe-frag.glsl, 134, GLSL )
#define RES_BLOOM_FRAG_GLSL                   CINDER_RESOURCE( ../resources/, bloom-frag.glsl, 135, GLSL )

// menu icons
#define RES_BRUSH_SVG                         CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 200, SVG )

#define RES_STRENGTH_SVG                      CINDER_RESOURCE( ../resources/menu-icons/, brush-strength-2.svg, 201, SVG )
#define RES_SIZE_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, brush-strength-3.svg, 202, SVG )
#define RES_TYPE_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, brush-type.svg, 203, SVG )

#define RES_PAINT_SVG                         CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 204, SVG ) // need new
#define RES_PUSH_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 205, SVG )
#define RES_SWEEP_SVG                         CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 206, SVG )
#define RES_FLATTEN_SVG                       CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 207, SVG )
#define RES_SMOOTH_SVG                        CINDER_RESOURCE( ../resources/menu-icons/, smooth.svg, 208, SVG )
#define RES_SHRINK_SVG                        CINDER_RESOURCE( ../resources/menu-icons/, shrink.svg, 209, SVG )
#define RES_GROW_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, grow.svg, 210, SVG )

#define RES_SIZE_AUTO_SVG                     CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 211, SVG )

#define RES_FONT_FREIGHTSANS_TTF              CINDER_RESOURCE( ../resources/, freigsanpromed-webfont.ttf, 212, TTF )
#define RES_FONT_FREIGHTSANSBOLD_TTF          CINDER_RESOURCE( ../resources/, freigsanprobold-webfont.ttf, 213, TTF )
