#include QMK_KEYBOARD_H
#include "debug.h"
#include "action_layer.h"
#include "version.h"
#include "keymap_german.h"
#include "keymap_nordic.h"
#include "keymap_french.h"
#include "keymap_spanish.h"
#include "keymap_hungarian.h"
#include "keymap_swedish.h"
#include "keymap_br_abnt2.h"
#include "keymap_canadian_multilingual.h"
#include "keymap_german_ch.h"
#include "keymap_jp.h"
#include "keymap_bepo.h"

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define _BLANK_ KC_TRANSPARENT


/*
* WORD DEFINITIONS for LAYER 3 LETTER KEYS
* From original Ergodox EZ Code Friendly Qwerty Layout:
* https://github.com/qmk/qmk_firmware/blob/master/layouts/community/ergodox/qwerty_code_friendly/readme.md
*/
#ifndef CFQ_WORD_A
#define CFQ_WORD_A ""
#endif
#ifndef CFQ_WORD_B
#define CFQ_WORD_B ""
#endif
#ifndef CFQ_WORD_C
#define CFQ_WORD_C ""
#endif
#ifndef CFQ_WORD_D
#define CFQ_WORD_D ""
#endif
#ifndef CFQ_WORD_E
#define CFQ_WORD_E ""
#endif
#ifndef CFQ_WORD_F
#define CFQ_WORD_F ""
#endif
#ifndef CFQ_WORD_G
#define CFQ_WORD_G ""
#endif
#ifndef CFQ_WORD_H
#define CFQ_WORD_H ""
#endif
#ifndef CFQ_WORD_I
#define CFQ_WORD_I ""
#endif
#ifndef CFQ_WORD_J
#define CFQ_WORD_J ""
#endif
#ifndef CFQ_WORD_K
#define CFQ_WORD_K ""
#endif
#ifndef CFQ_WORD_L
#define CFQ_WORD_L ""
#endif
#ifndef CFQ_WORD_M
#define CFQ_WORD_M ""
#endif
#ifndef CFQ_WORD_N
#define CFQ_WORD_N ""
#endif
#ifndef CFQ_WORD_O
#define CFQ_WORD_O ""
#endif
#ifndef CFQ_WORD_P
#define CFQ_WORD_P ""
#endif
#ifndef CFQ_WORD_Q
#define CFQ_WORD_Q ""
#endif
#ifndef CFQ_WORD_R
#define CFQ_WORD_R ""
#endif
#ifndef CFQ_WORD_S
#define CFQ_WORD_S ""
#endif
#ifndef CFQ_WORD_T
#define CFQ_WORD_T ""
#endif
#ifndef CFQ_WORD_U
#define CFQ_WORD_U ""
#endif
#ifndef CFQ_WORD_V
#define CFQ_WORD_V ""
#endif
#ifndef CFQ_WORD_W
#define CFQ_WORD_W ""
#endif
#ifndef CFQ_WORD_X
#define CFQ_WORD_X ""
#endif
#ifndef CFQ_WORD_Y
#define CFQ_WORD_Y ""
#endif
#ifndef CFQ_WORD_Z
#define CFQ_WORD_Z ""
#endif

/* lower and title capitals versions (setup at start). */
static char *cfq_word_lut[2][26] = {
  {
    CFQ_WORD_A, CFQ_WORD_B, CFQ_WORD_C, CFQ_WORD_D, CFQ_WORD_E, CFQ_WORD_F,
    CFQ_WORD_G, CFQ_WORD_H, CFQ_WORD_I, CFQ_WORD_J, CFQ_WORD_K, CFQ_WORD_L,
    CFQ_WORD_M, CFQ_WORD_N, CFQ_WORD_O, CFQ_WORD_P, CFQ_WORD_Q, CFQ_WORD_R,
    CFQ_WORD_S, CFQ_WORD_T, CFQ_WORD_U, CFQ_WORD_V, CFQ_WORD_W, CFQ_WORD_X,
    CFQ_WORD_Y, CFQ_WORD_Z,
  },
  {NULL}
};

/* Storage for title-caps strings. */
static char cfq_word_lut_title_caps[
    sizeof(CFQ_WORD_A) + sizeof(CFQ_WORD_B) + sizeof(CFQ_WORD_C) + sizeof(CFQ_WORD_D) +
    sizeof(CFQ_WORD_E) + sizeof(CFQ_WORD_F) + sizeof(CFQ_WORD_G) + sizeof(CFQ_WORD_H) +
    sizeof(CFQ_WORD_I) + sizeof(CFQ_WORD_J) + sizeof(CFQ_WORD_K) + sizeof(CFQ_WORD_L) +
    sizeof(CFQ_WORD_M) + sizeof(CFQ_WORD_N) + sizeof(CFQ_WORD_O) + sizeof(CFQ_WORD_P) +
    sizeof(CFQ_WORD_Q) + sizeof(CFQ_WORD_R) + sizeof(CFQ_WORD_S) + sizeof(CFQ_WORD_T) +
    sizeof(CFQ_WORD_U) + sizeof(CFQ_WORD_V) + sizeof(CFQ_WORD_W) + sizeof(CFQ_WORD_X) +
    sizeof(CFQ_WORD_Y) + sizeof(CFQ_WORD_Z)
];

#define LAYER_BASE 0 /* Default Layer. */
#define LAYER_KPAD 1 /* Keypad, Bracket Pairs & Macro Record. */
#define LAYER_FKEY 2 /* Function Keys, Media & Mouse Keys. */
#define LAYER_WORD 3 /* Entire Words (one for each key) & Numbers. */

enum custom_keycodes {
  PLACEHOLDER = SAFE_RANGE, /* can always be here */
  RGB_SLD,

  M_BRACKET_IN_CBR,
  M_BRACKET_IN_PRN,
  M_BRACKET_IN_BRC,
  M_BRACKET_IN_ANG,
  M_BRACKET_OUT_CBR,
  M_BRACKET_OUT_PRN,
  M_BRACKET_OUT_BRC,
  M_BRACKET_OUT_ANG,
  M_ARROW_RMINUS,
  M_ARROW_LMINUS,
  M_ARROW_REQL,
  M_ARROW_LEQL,

  /* allow user defined words for each character:
   * use CFQ_WORD_[A-Z] defines. */
  M_WORD_A, M_WORD_B, M_WORD_C, M_WORD_D, M_WORD_E, M_WORD_F,
  M_WORD_G, M_WORD_H, M_WORD_I, M_WORD_J, M_WORD_K, M_WORD_L,
  M_WORD_M, M_WORD_N, M_WORD_O, M_WORD_P, M_WORD_Q, M_WORD_R,
  M_WORD_S, M_WORD_T, M_WORD_U, M_WORD_V, M_WORD_W, M_WORD_X,
  M_WORD_Y, M_WORD_Z,

};


/* END MATERIAL COPIED FROM LEGACY LAYOUT */



const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT_ergodox_pretty(
    KC_EQUAL,       KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,        KC_LCBR,                                        KC_RCBR,        KC_CIRC,        KC_AMPR,        KC_KP_ASTERISK, KC_MINUS,       KC_EQUAL,       KC_BSPACE,
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,           KC_LPRN,                                        KC_RPRN,        KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           KC_BSLASH,
    KC_ESCAPE,      KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                                                           KC_H,           KC_J,           KC_K,           KC_L,           LT(2,KC_SCOLON),RGUI_T(KC_QUOTE),
    KC_LSHIFT,      LCTL_T(KC_Z),   KC_X,           KC_C,           KC_V,           KC_B,           MT(MOD_HYPR, KC_LBRACKET),                                MT(MOD_MEH, KC_RBRACKET),KC_N,           KC_M,           KC_COMMA,       KC_DOT,         RCTL_T(KC_SLASH),RSFT_T(KC_ENTER),
    LT(1,KC_GRAVE), _BLANK_, KC_LALT,        KC_LGUI,        WEBUSB_PAIR,                                                                                                    KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_DELETE,
                                                                                                    KC_BSPACE,      KC_DELETE,      KC_HOME,        KC_END,
                                                                                                                    KC_INSERT,      KC_PGUP,
                                                                                    LSFT_T(KC_ENTER),MO(1),          KC_INSERT,      KC_PGDOWN,      MO(2),          KC_SPACE
  ),
  [LAYER_KPAD] = LAYOUT_ergodox_pretty(
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,          M_BRACKET_IN_CBR,                    M_BRACKET_OUT_CBR, _BLANK_,   KC_NUMLOCK,     KC_KP_SLASH,    KC_KP_ASTERISK, KC_KP_MINUS,    _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, M_ARROW_RMINUS,   M_BRACKET_IN_PRN,                    M_BRACKET_OUT_PRN, M_ARROW_LEQL,     KC_KP_7,        KC_KP_8,        KC_KP_9,        KC_KP_PLUS,     _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, M_ARROW_REQL,                                                             M_ARROW_LMINUS,   KC_KP_4,        KC_KP_5,        KC_KP_6,        KC_KP_PLUS,     _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, M_BRACKET_IN_ANG, M_BRACKET_IN_BRC,                    M_BRACKET_OUT_BRC, M_BRACKET_OUT_ANG,KC_KP_1,        KC_KP_2,        KC_KP_3,        KC_KP_ENTER,    _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,                                                                                                 KC_KP_0,        _BLANK_, KC_KP_DOT,      KC_KP_ENTER,    _BLANK_,
                                        DYN_REC_START1, DYN_REC_START2, _BLANK_, _BLANK_,
                                                                                                                    DYN_MACRO_PLAY1,_BLANK_,
                                                                                    DYN_REC_STOP,   _BLANK_, DYN_MACRO_PLAY2,_BLANK_, MO(3),          _BLANK_
  ),
  [LAYER_FKEY] = LAYOUT_ergodox_pretty(
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,                                 KC_AUDIO_MUTE,    _BLANK_, KC_F10,         KC_F11,         KC_F12,         _BLANK_, RESET,
    _BLANK_, _BLANK_, _BLANK_, KC_MS_UP,       _BLANK_, _BLANK_, KC_MS_WH_UP,                                    KC_AUDIO_VOL_UP,  _BLANK_, KC_F7,          KC_F8,          KC_F9,          _BLANK_, _BLANK_,
    _BLANK_, _BLANK_, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    _BLANK_,                                                                   _BLANK_, KC_F4,          KC_F5,          KC_F6,          _BLANK_, KC_MEDIA_PLAY_PAUSE,
    _BLANK_, _BLANK_, KC_MS_BTN2,     KC_MS_BTN3,     KC_MS_BTN1,     _BLANK_, KC_MS_WH_DOWN,                                  KC_AUDIO_VOL_DOWN,_BLANK_, KC_F1,          KC_F2,          KC_F3,          _BLANK_, _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,                                                                                                   _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,
                                                                                                    _BLANK_, _BLANK_, KC_MEDIA_REWIND,KC_MEDIA_FAST_FORWARD,
                                                                                                                    _BLANK_, KC_MEDIA_PREV_TRACK,
                                                                                    _BLANK_, MO(3),          _BLANK_, KC_MEDIA_NEXT_TRACK,_BLANK_, KC_MEDIA_PLAY_PAUSE
  ),
  [LAYER_WORD] = LAYOUT_ergodox_pretty(
    _BLANK_, KC_1,     KC_2,     KC_3,     KC_4,     KC_5,     _BLANK_,                                 _BLANK_, KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     _BLANK_,
    _BLANK_, M_WORD_Q, M_WORD_W, M_WORD_E, M_WORD_R, M_WORD_T, _BLANK_,                                 _BLANK_, M_WORD_Y, M_WORD_U, M_WORD_I, M_WORD_O, M_WORD_P, _BLANK_,
    _BLANK_, M_WORD_A, M_WORD_S, M_WORD_D, M_WORD_F, M_WORD_G,                                                   M_WORD_H, M_WORD_J, M_WORD_K, M_WORD_L, _BLANK_,  _BLANK_,
    _BLANK_, M_WORD_Z, M_WORD_X, M_WORD_C, M_WORD_V, M_WORD_B, _BLANK_,                                 _BLANK_, M_WORD_N, M_WORD_M, _BLANK_,  _BLANK_,  _BLANK_,  _BLANK_,
    _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,                                                                     _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_,
                                                                 _BLANK_, _BLANK_, _BLANK_, _BLANK_,
                                                                          _BLANK_, _BLANK_,
                                                        _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_, _BLANK_
  ),
};


extern bool g_suspend_state;
extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [1] = { {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242}, {53,208,242} },

    [2] = { {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255}, {29,208,255} },

    [3] = { {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242}, {233,255,242} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < DRIVER_LED_TOTAL; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

void rgb_matrix_indicators_user(void) {
  if (g_suspend_state || keyboard_config.disable_layer_led) { return; }
  switch (biton32(layer_state)) {
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
}

#define WITHOUT_MODS(...) \
  do { \
    uint8_t _real_mods = get_mods(); \
    clear_mods(); \
    { __VA_ARGS__ } \
    set_mods(_real_mods); \
  } while (0)

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
      break;
    case M_BRACKET_IN_CBR:  /* {} */
      if (record->event.pressed) {
        SEND_STRING("{}" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_IN_PRN:  /* () */
      if (record->event.pressed) {
        SEND_STRING("()" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_IN_BRC:  /* [] */
      if (record->event.pressed) {
        SEND_STRING("[]" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_IN_ANG:  /* <> */
      if (record->event.pressed) {
        SEND_STRING("<>" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_OUT_CBR:  /* }{ */
      if (record->event.pressed) {
        SEND_STRING("}{" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_OUT_PRN:  /* )( */
      if (record->event.pressed) {
        SEND_STRING(")(" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_OUT_BRC:  /* ][ */
      if (record->event.pressed) {
        SEND_STRING("][" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_BRACKET_OUT_ANG:  /* >< */
      if (record->event.pressed) {
        SEND_STRING("><" SS_TAP(X_LEFT));
        return false;
      }
      break;
    case M_ARROW_LMINUS:  /* <- */
      if (record->event.pressed) {
        SEND_STRING("<-");
        return false;
      }
      break;
    case M_ARROW_RMINUS:  /* -> */
      if (record->event.pressed) {
        SEND_STRING("->");
        return false;
      }
      break;
    case M_ARROW_LEQL:  /* <= */
      if (record->event.pressed) {
        SEND_STRING("<=");
        return false;
      }
      break;
    case M_ARROW_REQL:  /* => */
      if (record->event.pressed) {
        SEND_STRING("=>");
        return false;
      }
      break;
#ifdef CFQ_USE_SHIFT_QUOTES
    case KC_LSHIFT:  /* '' */
      if (record->event.pressed && (keyboard_report->mods & (MOD_BIT(KC_RSFT)))) {
        WITHOUT_MODS({
            SEND_STRING("''" SS_TAP(X_LEFT) SS_DOWN(X_RSHIFT) SS_DOWN(X_LSHIFT));
          });
        return false;
      }
      break;
    case KC_RSHIFT:  /* "" */
      if (record->event.pressed && (keyboard_report->mods & (MOD_BIT(KC_LSFT)))) {
        WITHOUT_MODS({
            SEND_STRING("\x22\x22" SS_TAP(X_LEFT) SS_DOWN(X_LSHIFT) SS_DOWN(X_RSHIFT));
          });
        return false;
      }
      break;
#endif  /* CFQ_USE_SHIFT_QUOTES */
    case M_WORD_A...M_WORD_Z:
    {
      uint8_t shift_index = (keyboard_report->mods & (MOD_BIT(KC_RSFT) | MOD_BIT(KC_LSFT))) ? 1 : 0;
      const char *word = cfq_word_lut[shift_index][keycode - M_WORD_A];
      if (record->event.pressed) {
        if (*word) {
          WITHOUT_MODS({
              send_string(word);
            });
        }
        return false;
      }
      break;
    }
  }
  return true;
}

/* IMPORTED FROM ORIGINAL QMK QWERTY CODE KEYMAP */
/* Runs just one time when the keyboard initializes. */
void matrix_init_user(void) {

  /* Duplicate 'cfq_word_lut[0][...]' into 'cfq_word_lut[1][...]' */
  {
    char *d = cfq_word_lut_title_caps;
    for (uint16_t i = 0; i < 26; i++) {
      char *s = cfq_word_lut[0][i];
      cfq_word_lut[1][i] = d;
      while ((*d++ = *s++)) {}
    }
  }
  /* Title caps. */
  for (uint16_t i = 0; i < 26; i++) {
    char *w = cfq_word_lut[1][i];
    bool prev_is_alpha = false;
    if (*w) {
      while (*w) {
        bool is_lower = (*w >= 'a' && *w <= 'z');
        bool is_upper = (*w >= 'A' && *w <= 'Z');
        if (prev_is_alpha == false && is_lower) {
          *w -= ('a' - 'A');
        }
        prev_is_alpha = is_lower || is_upper;
        w++;
      }
    }
  }
}

/* END IMPORT */

uint32_t layer_state_set_user(uint32_t state) {

  uint8_t layer = biton32(state);

  ergodox_board_led_off();
  ergodox_right_led_1_off();
  ergodox_right_led_2_off();
  ergodox_right_led_3_off();
  switch (layer) {
    case 1:
      ergodox_right_led_1_on();
      break;
    case 2:
      ergodox_right_led_2_on();
      break;
    case 3:
      ergodox_right_led_3_on();
      break;
    case 4:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      break;
    case 5:
      ergodox_right_led_1_on();
      ergodox_right_led_3_on();
      break;
    case 6:
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    case 7:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    default:
      break;
  }
  return state;
};
