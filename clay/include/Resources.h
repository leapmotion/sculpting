#pragma once
#include "cinder/CinderResources.h"

// shaders
#define RES_SKY_VERT_GLSL                     CINDER_RESOURCE( ../resources/, sky-vert.glsl, 116, GLSL )
#define RES_SKY_FRAG_GLSL                     CINDER_RESOURCE( ../resources/, sky-frag.glsl, 117, GLSL )
#define RES_PASSTHROUGH_VERT_GLSL             CINDER_RESOURCE( ../resources/, passthrough-vert.glsl, 118, GLSL )
#define RES_MATERIAL_VERT_GLSL                CINDER_RESOURCE( ../resources/, material-vert.glsl, 120, GLSL )
#define RES_MATERIAL_FRAG_GLSL                CINDER_RESOURCE( ../resources/, material-frag.glsl, 121, GLSL )
#define RES_BRUSH_VERT_GLSL                   CINDER_RESOURCE( ../resources/, brush-vert.glsl, 122, GLSL )
#define RES_SCREEN_FRAG_GLSL                  CINDER_RESOURCE( ../resources/, screen-frag.glsl, 131, GLSL )
#define RES_FXAA_FRAG_GLSL                    CINDER_RESOURCE( ../resources/, fxaa-frag.glsl, 133, GLSL )
#define RES_WIREFRAME_FRAG_GLSL               CINDER_RESOURCE( ../resources/, wireframe-frag.glsl, 134, GLSL )
#define RES_BLOOM_FRAG_GLSL                   CINDER_RESOURCE( ../resources/, bloom-frag.glsl, 135, GLSL )

// menu icons
#define RES_BRUSH_SVG                         CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 200, SVG )
#define RES_STRENGTH_SVG                      CINDER_RESOURCE( ../resources/menu-icons/, brush-strength-2.svg, 201, SVG )
#define RES_SIZE_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, brush-strength-3.svg, 202, SVG )
#define RES_TYPE_SVG                          CINDER_RESOURCE( ../resources/menu-icons/, brush-type.svg, 203, SVG )
#define RES_PAINT_PNG                         CINDER_RESOURCE( ../resources/menu-icons/, paint.png, 204, PNG )
#define RES_PUSH_PNG                          CINDER_RESOURCE( ../resources/menu-icons/, press.png, 205, PNG )
#define RES_SWEEP_PNG                         CINDER_RESOURCE( ../resources/menu-icons/, sweep.png, 206, PNG )
#define RES_FLATTEN_PNG                       CINDER_RESOURCE( ../resources/menu-icons/, flatten.png, 207, PNG )
#define RES_SMOOTH_PNG                        CINDER_RESOURCE( ../resources/menu-icons/, smooth.png, 208, PNG )
#define RES_SHRINK_PNG                        CINDER_RESOURCE( ../resources/menu-icons/, repel.png, 209, PNG )
#define RES_GROW_PNG                          CINDER_RESOURCE( ../resources/menu-icons/, grow.png, 210, PNG )
#define RES_SIZE_AUTO_SVG                     CINDER_RESOURCE( ../resources/menu-icons/, brush.svg, 211, SVG )
#define RES_PAINT_SELECTED_PNG                CINDER_RESOURCE( ../resources/menu-icons/, paint-selected.png, 224, PNG )
#define RES_PUSH_SELECTED_PNG                 CINDER_RESOURCE( ../resources/menu-icons/, press-selected.png, 225, PNG )
#define RES_SWEEP_SELECTED_PNG                CINDER_RESOURCE( ../resources/menu-icons/, sweep-selected.png, 226, PNG )
#define RES_FLATTEN_SELECTED_PNG              CINDER_RESOURCE( ../resources/menu-icons/, flatten-selected.png, 227, PNG )
#define RES_SMOOTH_SELECTED_PNG               CINDER_RESOURCE( ../resources/menu-icons/, smooth-selected.png, 228, PNG )
#define RES_SHRINK_SELECTED_PNG               CINDER_RESOURCE( ../resources/menu-icons/, repel-selected.png, 229, PNG )
#define RES_GROW_SELECTED_PNG                 CINDER_RESOURCE( ../resources/menu-icons/, grow-selected.png, 230, PNG )
#define RES_STRENGTH_LOW_SELECTED_PNG         CINDER_RESOURCE( ../resources/menu-icons/, strength-low-selected.png, 231, PNG )
#define RES_STRENGTH_MEDIUM_SELECTED_PNG      CINDER_RESOURCE( ../resources/menu-icons/, strength-medium-selected.png, 232, PNG )
#define RES_STRENGTH_HIGH_SELECTED_PNG        CINDER_RESOURCE( ../resources/menu-icons/, strength-high-selected.png, 233, PNG )

// material icons
#define RES_PLASTIC_PNG                       CINDER_RESOURCE( ../resources/menu-icons/, plastic.png, 235, PNG )
#define RES_PORCELAIN_PNG                     CINDER_RESOURCE( ../resources/menu-icons/, porcelain.png, 236, PNG )
#define RES_STEEL_PNG                         CINDER_RESOURCE( ../resources/menu-icons/, steel.png, 237, PNG )
#define RES_CLAY_PNG                          CINDER_RESOURCE( ../resources/menu-icons/, clay.png, 238, PNG )
#define RES_GLASS_PNG                         CINDER_RESOURCE( ../resources/menu-icons/, glass.png, 239, PNG )

// fonts
#define RES_FONT_FREIGHTSANS_TTF              CINDER_RESOURCE( ../resources/, freigsanpromed-webfont.ttf, 212, TTF )
#define RES_FONT_FREIGHTSANSBOLD_TTF          CINDER_RESOURCE( ../resources/, freigsanprobold-webfont.ttf, 213, TTF )

// shapes
#define RES_BALL_OBJ                          CINDER_RESOURCE( ../resources/shapes/, ball.obj, 250, OBJ )
#define RES_CAN_OBJ                           CINDER_RESOURCE( ../resources/shapes/, can.obj, 251, OBJ )
#define RES_DONUT_OBJ                         CINDER_RESOURCE( ../resources/shapes/, donut.obj, 252, OBJ )
#define RES_SHEET_OBJ                         CINDER_RESOURCE( ../resources/shapes/, sheet.obj, 253, OBJ )

// tutorial images
#define RES_TUTORIAL_1                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-camera.png, 260, PNG )
#define RES_TUTORIAL_2                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-menu.png, 261, PNG )
#define RES_TUTORIAL_3                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-tools.png, 262, PNG )
