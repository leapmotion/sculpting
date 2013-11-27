#pragma once
#include "cinder/CinderResources.h"

#define ICON_IDX 1

// shaders
#define RES_SKY_VERT_GLSL                     CINDER_RESOURCE( ../resources/, sky-vert.glsl, 116, GLSL )
#define RES_SKY_FRAG_GLSL                     CINDER_RESOURCE( ../resources/, sky-frag.glsl, 117, GLSL )
#define RES_PASSTHROUGH_VERT_GLSL             CINDER_RESOURCE( ../resources/, passthrough-vert.glsl, 118, GLSL )
#define RES_MATERIAL_VERT_GLSL                CINDER_RESOURCE( ../resources/, material-vert.glsl, 120, GLSL )
#define RES_MATERIAL_FRAG_GLSL                CINDER_RESOURCE( ../resources/, material-frag.glsl, 121, GLSL )
#define RES_BRUSH_VERT_GLSL                   CINDER_RESOURCE( ../resources/, brush-vert.glsl, 122, GLSL )
#define RES_SCREEN_FRAG_GLSL                  CINDER_RESOURCE( ../resources/, screen-frag.glsl, 131, GLSL )
#define RES_WIREFRAME_FRAG_GLSL               CINDER_RESOURCE( ../resources/, wireframe-frag.glsl, 134, GLSL )
#define RES_BLOOM_FRAG_GLSL                   CINDER_RESOURCE( ../resources/, bloom-frag.glsl, 135, GLSL )
#define RES_PREVIEWS_FRAG_GLSL                CINDER_RESOURCE( ../resources/, previews-frag.glsl, 146, GLSL )

// menu icons
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
#define RES_CUBE_OBJ                          CINDER_RESOURCE( ../resources/shapes/, cube.obj, 254, OBJ )
#define RES_TURKEY_OBJ                        CINDER_RESOURCE( ../resources/shapes/, turkey.obj, 255, OBJ )

// tutorial images
#define RES_TUTORIAL_1                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-menu.png, 260, PNG )
#define RES_TUTORIAL_2                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-camera.png, 261, PNG )
#define RES_TUTORIAL_3                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-tools.png, 262, PNG )
#define RES_TUTORIAL_4                        CINDER_RESOURCE( ../resources/tutorial/, tutorial-immersive.png, 263, PNG )

// logo images
#define RES_LOGO_ON_BLACK                     CINDER_RESOURCE( ../resources/logos/, freeform-on-black.png, 270, PNG )
#define RES_LOGO_ON_IMAGE                     CINDER_RESOURCE( ../resources/logos/, freeform-on-image.png, 271, PNG )

// about image
#define RES_CREDITS                           CINDER_RESOURCE( ../resources/, credits.png, 280, PNG )

// environment previews
#define RES_PREVIEW_JUNGLE_CLIFF              CINDER_RESOURCE( ../resources/previews/, jungle-cliff.png, 290, PNG )
#define RES_PREVIEW_JUNGLE                    CINDER_RESOURCE( ../resources/previews/, jungle.png, 291, PNG )
#define RES_PREVIEW_ISLANDS                   CINDER_RESOURCE( ../resources/previews/, islands.png, 292, PNG )
#define RES_PREVIEW_REDWOOD                   CINDER_RESOURCE( ../resources/previews/, redwood.png, 293, PNG )
#define RES_PREVIEW_DESERT                    CINDER_RESOURCE( ../resources/previews/, desert.png, 294, PNG )
#define RES_PREVIEW_ARCTIC                    CINDER_RESOURCE( ../resources/previews/, arctic.png, 295, PNG )
#define RES_PREVIEW_RIVER                     CINDER_RESOURCE( ../resources/previews/, river.png, 296, PNG )
