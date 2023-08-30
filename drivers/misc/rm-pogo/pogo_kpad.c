// SPDX-License-Identifier: GPL-2.0-only
/*
 * reMarkable POGO keyboard map
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "pogo.h"
#include <linux/input/matrix_keypad.h>

#define MAX_MATRIX_KEY_ROWS 7
#define MAX_MATRIX_KEY_COLS 15

/* Seabird_pC08_20220321, with row0 not connected. The row0 here is row1 on PCB */
static const uint32_t pogo_keymap_proto1[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_F1),
	KEY(0, 2, KEY_F5),
	KEY(0, 3, KEY_G),
	KEY(0, 4, KEY_F7),
	KEY(0, 5, KEY_RESERVED),
	KEY(0, 6, KEY_H),
	KEY(0, 7, KEY_RESERVED),
	KEY(0, 8, KEY_APOSTROPHE),
	KEY(0, 9, KEY_F10),
	KEY(0, 10, KEY_RESERVED),
	KEY(0, 11, KEY_BACKSPACE),
	KEY(0, 12, KEY_RESERVED),	/* not found in kb */
	KEY(0, 13, KEY_KPASTERISK),
	KEY(0, 14, KEY_KP8),
	KEY(1, 0, KEY_LEFTCTRL),
	KEY(1, 1, KEY_TAB),
	KEY(1, 2, KEY_F4),
	KEY(1, 3, KEY_T),
	KEY(1, 4, KEY_F7),
	KEY(1, 5, KEY_RIGHTBRACE),
	KEY(1, 6, KEY_Y),
	KEY(1, 7, KEY_RESERVED),	/* not found in kb */
	KEY(1, 8, KEY_LEFTBRACE),
	KEY(1, 9, KEY_F9),
	KEY(1, 10, KEY_RESERVED),	/* not found in kb */
	KEY(1, 11, KEY_DELETE),
	KEY(1, 12, KEY_PAGEUP),
	KEY(1, 13, KEY_KPMINUS),
	KEY(1, 14, KEY_KP9),
	KEY(2, 0, KEY_LEFTCTRL),
	KEY(2, 1, KEY_GRAVE),
	KEY(2, 2, KEY_F3),
	KEY(2, 3, KEY_5),
	KEY(2, 4, KEY_F6),
	KEY(2, 5, KEY_RESERVED),
	KEY(2, 6, KEY_6),
	KEY(2, 7, KEY_RESERVED),
	KEY(2, 8, KEY_MINUS),
	KEY(2, 9, KEY_SCREENLOCK),
	KEY(2, 10, KEY_RESERVED),
	KEY(2, 11, KEY_BACKSLASH),
	KEY(2, 12, KEY_RESERVED),
	KEY(2, 13, KEY_KP6),
	KEY(2, 14, KEY_KPPLUS),
	KEY(3, 0, KEY_RIGHTCTRL),
	KEY(3, 1, KEY_A),
	KEY(3, 2, KEY_D),
	KEY(3, 3, KEY_F),
	KEY(3, 4, KEY_S),
	KEY(3, 5, KEY_K),
	KEY(3, 6, KEY_J),
	KEY(3, 7, KEY_RESERVED),
	KEY(3, 8, KEY_SEMICOLON),
	KEY(3, 9, KEY_L),
	KEY(3, 10, KEY_RESERVED),	/* not found */
	KEY(3, 11, KEY_ENTER),
	KEY(3, 12, KEY_END),
	KEY(3, 13, KEY_KPDOT),
	KEY(3, 14, KEY_KP3),
	KEY(4, 0, KEY_RESERVED),	/* not found */
	KEY(4, 1, KEY_Z),
	KEY(4, 2, KEY_C),
	KEY(4, 3, KEY_V),
	KEY(4, 4, KEY_X),
	KEY(4, 5, KEY_COMMA),
	KEY(4, 6, KEY_M),
	KEY(4, 7, KEY_LEFTSHIFT),
	KEY(4, 8, KEY_SLASH),
	KEY(4, 9, KEY_DOT),
	KEY(4, 10, KEY_RESERVED),
	KEY(4, 11, KEY_SPACE),
	KEY(4, 12, KEY_PAGEDOWN),
	KEY(4, 13, KEY_KP5),
	KEY(4, 14, KEY_KP2),
	KEY(5, 0, KEY_RESERVED),
	KEY(5, 1, KEY_1),
	KEY(5, 2, KEY_3),
	KEY(5, 3, KEY_4),
	KEY(5, 4, KEY_2),
	KEY(5, 5, KEY_8),
	KEY(5, 6, KEY_7),
	KEY(5, 7, KEY_RESERVED),
	KEY(5, 8, KEY_0),
	KEY(5, 9, KEY_9),
	KEY(5, 10, KEY_RIGHTALT),
	KEY(5, 11, KEY_DOWN),
	KEY(5, 12, KEY_RIGHT),
	KEY(5, 13, KEY_KP4),
	KEY(5, 14, KEY_KP1),
	KEY(6, 0, KEY_RESERVED),
	KEY(6, 1, KEY_Q),
	KEY(6, 2, KEY_E),
	KEY(6, 3, KEY_R),
	KEY(6, 4, KEY_W),
	KEY(6, 5, KEY_I),
	KEY(6, 6, KEY_U),
	KEY(6, 7, KEY_RIGHTSHIFT),
	KEY(6, 8, KEY_P),
	KEY(6, 9, KEY_O),
	KEY(6, 10, KEY_RESERVED),
	KEY(6, 11, KEY_UP),
	KEY(6, 12, KEY_LEFT),
	KEY(6, 13, KEY_KP0),
	KEY(6, 14, KEY_KPENTER),
};

/* Proto#3, Seabird_2022 0602 */
static const uint32_t pogo_keymap_proto3[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_RESERVED),
	KEY(0, 1, KEY_RESERVED),
	KEY(0, 2, KEY_RESERVED),
	KEY(0, 3, KEY_RIGHT),
	KEY(0, 4, KEY_RESERVED),
	KEY(0, 5, KEY_RESERVED),
	KEY(0, 6, KEY_RESERVED),
	KEY(0, 7, KEY_B),
	KEY(0, 8, KEY_X),
	KEY(0, 9, KEY_K),
	KEY(0, 10, KEY_E),
	KEY(0, 11, KEY_Q),
	KEY(0, 12, KEY_SEMICOLON),
	KEY(0, 13, KEY_RESERVED),
	KEY(0, 14, KEY_LEFTSHIFT),
	KEY(1, 0, KEY_RESERVED),
	KEY(1, 1, KEY_RESERVED),
	KEY(1, 2, KEY_RESERVED),
	KEY(1, 3, KEY_RESERVED),
	KEY(1, 4, KEY_DOWN),
	KEY(1, 5, KEY_RESERVED),
	KEY(1, 6, KEY_RESERVED),
	KEY(1, 7, KEY_D),
	KEY(1, 8, KEY_I),
	KEY(1, 9, KEY_U),
	KEY(1, 10, KEY_J),
	KEY(1, 11, KEY_RESERVED),
	KEY(1, 12, KEY_RESERVED),
	KEY(1, 13, KEY_END),
	KEY(1, 14, KEY_RESERVED),
	KEY(2, 0, KEY_LEFTCTRL),
	KEY(2, 1, KEY_LEFTALT),
	KEY(2, 2, KEY_RESERVED),
	KEY(2, 3, KEY_BACKSPACE),
	KEY(2, 4, KEY_EQUAL),
	KEY(2, 5, KEY_LEFT),
	KEY(2, 6, KEY_ENTER),
	KEY(2, 7, KEY_7),
	KEY(2, 8, KEY_Y),
	KEY(2, 9, KEY_DOT),
	KEY(2, 10, KEY_P),
	KEY(2, 11, KEY_COMMA),
	KEY(2, 12, KEY_APOSTROPHE),
	KEY(2, 13, KEY_RESERVED),
	KEY(2, 14, KEY_RESERVED),
	KEY(3, 0, KEY_RESERVED),
	KEY(3, 1, KEY_RESERVED),
	KEY(3, 2, KEY_RESERVED),
	KEY(3, 3, KEY_LEFTBRACE),
	KEY(3, 4, KEY_0),
	KEY(3, 5, KEY_C),
	KEY(3, 6, KEY_UP),
	KEY(3, 7, KEY_F),
	KEY(3, 8, KEY_6),
	KEY(3, 9, KEY_5),
	KEY(3, 10, KEY_4),
	KEY(3, 11, KEY_3),
	KEY(3, 12, KEY_LEFTCTRL),
	KEY(3, 13, KEY_O),
	KEY(3, 14, KEY_A),
	KEY(4, 0, KEY_RESERVED),
	KEY(4, 1, KEY_RESERVED),
	KEY(4, 2, KEY_HOME),
	KEY(4, 3, KEY_L),
	KEY(4, 4, KEY_R),
	KEY(4, 5, KEY_T),
	KEY(4, 6, KEY_G),
	KEY(4, 7, KEY_8),
	KEY(4, 8, KEY_RESERVED),
	KEY(4, 9, KEY_RESERVED),
	KEY(4, 10, KEY_RESERVED),
	KEY(4, 11, KEY_2),
	KEY(4, 12, KEY_1),
	KEY(4, 13, KEY_RESERVED),
	KEY(4, 14, KEY_RESERVED),
	KEY(5, 0, KEY_RESERVED),
	KEY(5, 1, KEY_RESERVED),
	KEY(5, 2, KEY_RESERVED),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_N),
	KEY(5, 5, KEY_9),
	KEY(5, 6, KEY_H),
	KEY(5, 7, KEY_SLASH),
	KEY(5, 8, KEY_RESERVED),
	KEY(5, 9, KEY_RESERVED),
	KEY(5, 10, KEY_RESERVED),
	KEY(5, 11, KEY_RESERVED),
	KEY(5, 12, KEY_TAB),
	KEY(5, 13, KEY_RESERVED),
	KEY(5, 14, KEY_RIGHTSHIFT),
	KEY(6, 0, KEY_RESERVED),
	KEY(6, 1, KEY_RIGHTALT),
	KEY(6, 2, KEY_SPACE),
	KEY(6, 3, KEY_Z),
	KEY(6, 4, KEY_V),
	KEY(6, 5, KEY_W),
	KEY(6, 6, KEY_M),
	KEY(6, 7, KEY_MINUS),
	KEY(6, 8, KEY_RESERVED),
	KEY(6, 9, KEY_RESERVED),
	KEY(6, 10, KEY_RESERVED),
	KEY(6, 11, KEY_RESERVED),
	KEY(6, 12, KEY_RESERVED),
	KEY(6, 13, KEY_RESERVED),
	KEY(6, 14, KEY_RESERVED),

	/* layer 1 */
	KEY(7, 0, KEY_RESERVED),
	KEY(7, 1, KEY_RESERVED),
	KEY(7, 2, KEY_RESERVED),
	KEY(7, 3, KEY_END),
	KEY(7, 4, KEY_RESERVED),
	KEY(7, 5, KEY_RESERVED),
	KEY(7, 6, KEY_RESERVED),
	KEY(7, 7, KEY_B),
	KEY(7, 8, KEY_X),
	KEY(7, 9, KEY_K),
	KEY(7, 10, KEY_E),
	KEY(7, 11, KEY_Q),
	KEY(7, 12, KEY_SEMICOLON),
	KEY(7, 13, KEY_RESERVED),
	KEY(7, 14, KEY_LEFTSHIFT),
	KEY(8, 0, KEY_RESERVED),
	KEY(8, 1, KEY_RESERVED),
	KEY(8, 2, KEY_RESERVED),
	KEY(8, 3, KEY_RESERVED),
	KEY(8, 4, KEY_PAGEDOWN),
	KEY(8, 5, KEY_RESERVED),
	KEY(8, 6, KEY_RESERVED),
	KEY(8, 7, KEY_D),
	KEY(8, 8, KEY_I),
	KEY(8, 9, KEY_U),
	KEY(8, 10, KEY_J),
	KEY(8, 11, KEY_RESERVED),
	KEY(8, 12, KEY_RESERVED),
	KEY(8, 13, KEY_END),
	KEY(8, 14, KEY_RESERVED),
	KEY(9, 0, KEY_LEFTCTRL),
	KEY(9, 1, KEY_LEFTALT),
	KEY(9, 2, KEY_RESERVED),
	KEY(9, 3, KEY_DELETE),
	KEY(9, 4, KEY_BACKSLASH),
	KEY(9, 5, KEY_HOME),
	KEY(9, 6, KEY_ENTER),
	KEY(9, 7, KEY_7),
	KEY(9, 8, KEY_Y),
	KEY(9, 9, KEY_DOT),
	KEY(9, 10, KEY_P),
	KEY(9, 11, KEY_COMMA),
	KEY(9, 12, KEY_APOSTROPHE),
	KEY(9, 13, KEY_RESERVED),
	KEY(9, 14, KEY_RESERVED),
	KEY(10, 0, KEY_RESERVED),
	KEY(10, 1, KEY_RESERVED),
	KEY(10, 2, KEY_RESERVED),
	KEY(10, 3, KEY_RIGHTBRACE),
	KEY(10, 4, KEY_0),
	KEY(10, 5, KEY_C),
	KEY(10, 6, KEY_PAGEUP),
	KEY(10, 7, KEY_F),
	KEY(10, 8, KEY_6),
	KEY(10, 9, KEY_5),
	KEY(10, 10, KEY_4),
	KEY(10, 11, KEY_3),
	KEY(10, 12, KEY_LEFTCTRL),
	KEY(10, 13, KEY_O),
	KEY(10, 14, KEY_A),
	KEY(11, 0, KEY_RESERVED),
	KEY(11, 1, KEY_RESERVED),
	KEY(11, 2, KEY_HOME),
	KEY(11, 3, KEY_L),
	KEY(11, 4, KEY_R),
	KEY(11, 5, KEY_T),
	KEY(11, 6, KEY_G),
	KEY(11, 7, KEY_8),
	KEY(11, 8, KEY_RESERVED),
	KEY(11, 9, KEY_RESERVED),
	KEY(11, 10, KEY_RESERVED),
	KEY(11, 11, KEY_2),
	KEY(11, 12, KEY_ESC),
	KEY(11, 13, KEY_RESERVED),
	KEY(11, 14, KEY_RESERVED),
	KEY(12, 0, KEY_RESERVED),
	KEY(12, 1, KEY_RESERVED),
	KEY(12, 2, KEY_RESERVED),
	KEY(12, 3, KEY_S),
	KEY(12, 4, KEY_N),
	KEY(12, 5, KEY_9),
	KEY(12, 6, KEY_H),
	KEY(12, 7, KEY_SLASH),
	KEY(12, 8, KEY_RESERVED),
	KEY(12, 9, KEY_RESERVED),
	KEY(12, 10, KEY_RESERVED),
	KEY(12, 11, KEY_RESERVED),
	KEY(12, 12, KEY_GRAVE),
	KEY(12, 13, KEY_RESERVED),
	KEY(12, 14, KEY_RIGHTSHIFT),
	KEY(13, 0, KEY_RESERVED),
	KEY(13, 1, KEY_RIGHTALT),
	KEY(13, 2, KEY_SPACE),
	KEY(13, 3, KEY_Z),
	KEY(13, 4, KEY_V),
	KEY(13, 5, KEY_W),
	KEY(13, 6, KEY_M),
	KEY(13, 7, KEY_MINUS),
	KEY(13, 8, KEY_RESERVED),
	KEY(13, 9, KEY_RESERVED),
	KEY(13, 10, KEY_RESERVED),
	KEY(13, 11, KEY_RESERVED),
	KEY(13, 12, KEY_RESERVED),
	KEY(13, 13, KEY_RESERVED),
	KEY(13, 14, KEY_RESERVED),

	/* layer 2 */
	KEY(14, 0, KEY_RESERVED),
	KEY(14, 1, KEY_RESERVED),
	KEY(14, 2, KEY_RESERVED),
	KEY(14, 3, KEY_RIGHT),
	KEY(14, 4, KEY_RESERVED),
	KEY(14, 5, KEY_RESERVED),
	KEY(14, 6, KEY_RESERVED),
	KEY(14, 7, KEY_B),
	KEY(14, 8, KEY_X),
	KEY(14, 9, KEY_K),
	KEY(14, 10, KEY_E),
	KEY(14, 11, KEY_Q),
	KEY(14, 12, KEY_SEMICOLON),
	KEY(14, 13, KEY_RESERVED),
	KEY(14, 14, KEY_LEFTSHIFT),
	KEY(15, 0, KEY_RESERVED),
	KEY(15, 1, KEY_RESERVED),
	KEY(15, 2, KEY_RESERVED),
	KEY(15, 3, KEY_RESERVED),
	KEY(15, 4, KEY_DOWN),
	KEY(15, 5, KEY_RESERVED),
	KEY(15, 6, KEY_RESERVED),
	KEY(15, 7, KEY_D),
	KEY(15, 8, KEY_I),
	KEY(15, 9, KEY_U),
	KEY(15, 10, KEY_J),
	KEY(15, 11, KEY_RESERVED),
	KEY(15, 12, KEY_RESERVED),
	KEY(15, 13, KEY_END),
	KEY(15, 14, KEY_RESERVED),
	KEY(16, 0, KEY_LEFTCTRL),
	KEY(16, 1, KEY_LEFTALT),
	KEY(16, 2, KEY_RESERVED),
	KEY(16, 3, KEY_F12),
	KEY(16, 4, KEY_EQUAL),
	KEY(16, 5, KEY_LEFT),
	KEY(16, 6, KEY_ENTER),
	KEY(16, 7, KEY_F7),
	KEY(16, 8, KEY_Y),
	KEY(16, 9, KEY_DOT),
	KEY(16, 10, KEY_P),
	KEY(16, 11, KEY_COMMA),
	KEY(16, 12, KEY_APOSTROPHE),
	KEY(16, 13, KEY_RESERVED),
	KEY(16, 14, KEY_RESERVED),
	KEY(17, 0, KEY_RESERVED),
	KEY(17, 1, KEY_RESERVED),
	KEY(17, 2, KEY_RESERVED),
	KEY(17, 3, KEY_F11),
	KEY(17, 4, KEY_F10),
	KEY(17, 5, KEY_C),
	KEY(17, 6, KEY_UP),
	KEY(17, 7, KEY_F),
	KEY(17, 8, KEY_F6),
	KEY(17, 9, KEY_F5),
	KEY(17, 10, KEY_F4),
	KEY(17, 11, KEY_F3),
	KEY(17, 12, KEY_LEFTCTRL),
	KEY(17, 13, KEY_O),
	KEY(17, 14, KEY_A),
	KEY(18, 0, KEY_RESERVED),
	KEY(18, 1, KEY_RESERVED),
	KEY(18, 2, KEY_HOME),
	KEY(18, 3, KEY_L),
	KEY(18, 4, KEY_R),
	KEY(18, 5, KEY_T),
	KEY(18, 6, KEY_G),
	KEY(18, 7, KEY_F8),
	KEY(18, 8, KEY_RESERVED),
	KEY(18, 9, KEY_RESERVED),
	KEY(18, 10, KEY_RESERVED),
	KEY(18, 11, KEY_F2),
	KEY(18, 12, KEY_F1),
	KEY(18, 13, KEY_RESERVED),
	KEY(18, 14, KEY_RESERVED),
	KEY(19, 0, KEY_RESERVED),
	KEY(19, 1, KEY_RESERVED),
	KEY(19, 2, KEY_RESERVED),
	KEY(19, 3, KEY_S),
	KEY(19, 4, KEY_N),
	KEY(19, 5, KEY_F9),
	KEY(19, 6, KEY_H),
	KEY(19, 7, KEY_SLASH),
	KEY(19, 8, KEY_RESERVED),
	KEY(19, 9, KEY_RESERVED),
	KEY(19, 10, KEY_RESERVED),
	KEY(19, 11, KEY_RESERVED),
	KEY(19, 12, KEY_TAB),
	KEY(19, 13, KEY_RESERVED),
	KEY(19, 14, KEY_RIGHTSHIFT),
	KEY(20, 0, KEY_RESERVED),
	KEY(20, 1, KEY_RIGHTALT),
	KEY(20, 2, KEY_SPACE),
	KEY(20, 3, KEY_Z),
	KEY(20, 4, KEY_V),
	KEY(20, 5, KEY_W),
	KEY(20, 6, KEY_M),
	KEY(20, 7, KEY_MINUS),
	KEY(20, 8, KEY_RESERVED),
	KEY(20, 9, KEY_RESERVED),
	KEY(20, 10, KEY_RESERVED),
	KEY(20, 11, KEY_RESERVED),
	KEY(20, 12, KEY_RESERVED),
	KEY(20, 13, KEY_RESERVED),
	KEY(20, 14, KEY_RESERVED),
};

/* Debug for Proto#3, Seabird_2022 0511 */
static const uint32_t pogo_keymap_debug[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 0, KEY_A),
	KEY(0, 1, KEY_B),
	KEY(0, 2, KEY_C),
	KEY(0, 3, KEY_D),
	KEY(0, 4, KEY_E),
	KEY(0, 5, KEY_F),
	KEY(0, 6, KEY_G),
	KEY(0, 7, KEY_H),
	KEY(0, 8, KEY_I),
	KEY(0, 9, KEY_J),
	KEY(0, 10, KEY_K),
	KEY(0, 11, KEY_L),
	KEY(0, 12, KEY_M),
	KEY(0, 13, KEY_N),
	KEY(0, 14, KEY_O),
	KEY(1, 0, KEY_P),
	KEY(1, 1, KEY_Q),
	KEY(1, 2, KEY_R),
	KEY(1, 3, KEY_S),
	KEY(1, 4, KEY_T),
	KEY(1, 5, KEY_U),
	KEY(1, 6, KEY_V),
	KEY(1, 7, KEY_W),
	KEY(1, 8, KEY_X),
	KEY(1, 9, KEY_Y),
	KEY(1, 10, KEY_Z),
	KEY(1, 11, KEY_0),
	KEY(1, 12, KEY_1),
	KEY(1, 13, KEY_2),
	KEY(1, 14, KEY_3),
	KEY(2, 0, KEY_4),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_6),
	KEY(2, 3, KEY_7),
	KEY(2, 4, KEY_8),
	KEY(2, 5, KEY_9),
	KEY(2, 6, KEY_A),
	KEY(2, 7, KEY_B),
	KEY(2, 8, KEY_C),
	KEY(2, 9, KEY_D),
	KEY(2, 10, KEY_E),
	KEY(2, 11, KEY_F),
	KEY(2, 12, KEY_G),
	KEY(2, 13, KEY_H),
	KEY(2, 14, KEY_I),
	KEY(3, 0, KEY_J),
	KEY(3, 1, KEY_K),
	KEY(3, 2, KEY_L),
	KEY(3, 3, KEY_M),
	KEY(3, 4, KEY_N),
	KEY(3, 5, KEY_O),
	KEY(3, 6, KEY_P),
	KEY(3, 7, KEY_Q),
	KEY(3, 8, KEY_R),
	KEY(3, 9, KEY_S),
	KEY(3, 10, KEY_T),
	KEY(3, 11, KEY_U),
	KEY(3, 12, KEY_V),
	KEY(3, 13, KEY_W),
	KEY(3, 14, KEY_X),
	KEY(4, 0, KEY_Y),
	KEY(4, 1, KEY_Z),
	KEY(4, 2, KEY_0),
	KEY(4, 3, KEY_1),
	KEY(4, 4, KEY_2),
	KEY(4, 5, KEY_3),
	KEY(4, 6, KEY_4),
	KEY(4, 7, KEY_5),
	KEY(4, 8, KEY_6),
	KEY(4, 9, KEY_7),
	KEY(4, 10, KEY_8),
	KEY(4, 11, KEY_9),
	KEY(4, 12, KEY_A),
	KEY(4, 13, KEY_B),
	KEY(4, 14, KEY_C),
	KEY(5, 0, KEY_D),
	KEY(5, 1, KEY_E),
	KEY(5, 2, KEY_F),
	KEY(5, 3, KEY_G),
	KEY(5, 4, KEY_H),
	KEY(5, 5, KEY_I),
	KEY(5, 6, KEY_J),
	KEY(5, 7, KEY_K),
	KEY(5, 8, KEY_L),
	KEY(5, 9, KEY_M),
	KEY(5, 10, KEY_N),
	KEY(5, 11, KEY_O),
	KEY(5, 12, KEY_P),
	KEY(5, 13, KEY_Q),
	KEY(5, 14, KEY_R),
	KEY(6, 0, KEY_S),
	KEY(6, 1, KEY_T),
	KEY(6, 2, KEY_U),
	KEY(6, 3, KEY_V),
	KEY(6, 4, KEY_W),
	KEY(6, 5, KEY_X),
	KEY(6, 6, KEY_Y),
	KEY(6, 7, KEY_Z),
	KEY(6, 8, KEY_0),
	KEY(6, 9, KEY_1),
	KEY(6, 10, KEY_2),
	KEY(6, 11, KEY_3),
	KEY(6, 12, KEY_4),
	KEY(6, 13, KEY_5),
	KEY(6, 14, KEY_6),
};

/* MIRRORED FOR PCBA rev C! - Seabird_2022 0602 */
static const uint32_t pogo_keymap_proto3_type_c[] = {
    /* KEY(row, col, key_code) */
	KEY(0, 0, KEY_TAB),
    KEY(0, 1, KEY_RESERVED),
    KEY(0, 2, KEY_RESERVED),
	KEY(0, 3, KEY_RESERVED),
    KEY(0, 4, KEY_RESERVED),
	KEY(0, 5, KEY_9),
    KEY(0, 6, KEY_J),
	KEY(0, 7, KEY_SEMICOLON),
	KEY(0, 8, KEY_RESERVED),
    KEY(0, 9, KEY_RESERVED),
    KEY(0, 10, KEY_L),
	KEY(0, 11, KEY_RESERVED),
    KEY(0, 12, KEY_GRAVE),
	KEY(0, 13, KEY_RESERVED),
    KEY(0, 14, KEY_RIGHTSHIFT),	
	KEY(1, 0, KEY_Q),
	KEY(1, 1, KEY_T),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_W),
	KEY(1, 4, KEY_R),
	KEY(1, 5, KEY_LEFT),
	KEY(1, 6, KEY_ENTER),
	KEY(1, 7, KEY_BACKSPACE),
	KEY(1, 8, KEY_RESERVED),
	KEY(1, 9, KEY_LEFTALT),
    KEY(1, 10, KEY_BACKSLASH),
	KEY(1, 11, KEY_LEFTCTRL),
    KEY(1, 12, KEY_7),
	KEY(1, 13, KEY_RESERVED),
	KEY(1, 14, KEY_RESERVED),
	KEY(2, 0, KEY_RESERVED),
    KEY(2, 1, KEY_G),
    KEY(2, 2, KEY_F),
	KEY(2, 3, KEY_RESERVED),
    KEY(2, 4, KEY_C),
	KEY(2, 5, KEY_RESERVED),
	KEY(2, 6, KEY_RESERVED),
	KEY(2, 7, KEY_RESERVED),
	KEY(2, 8, KEY_RESERVED),
	KEY(2, 9, KEY_RESERVED),
	KEY(2, 10, KEY_DOWN),
	KEY(2, 11, KEY_RESERVED),
    KEY(2, 12, KEY_H),
    KEY(2, 13, KEY_END),
    KEY(2, 14, KEY_RESERVED),
	KEY(3, 0, KEY_1),
    KEY(3, 1, KEY_RESERVED),
    KEY(3, 2, KEY_RESERVED),
    KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_RESERVED),
    KEY(3, 5, KEY_K),
    KEY(3, 6, KEY_U),
	KEY(3, 7, KEY_P),
    KEY(3, 8, KEY_HOME),
	KEY(3, 9, KEY_RESERVED),
    KEY(3, 10, KEY_O),
	KEY(3, 11, KEY_RESERVED),
    KEY(3, 12, KEY_8),
	KEY(3, 13, KEY_RESERVED),
	KEY(3, 14, KEY_RESERVED),
	KEY(4, 0, KEY_LEFTCTRL),
	KEY(4, 1, KEY_6),
    KEY(4, 2, KEY_5),
	KEY(4, 3, KEY_3),
	KEY(4, 4, KEY_4),
    KEY(4, 5, KEY_I),
    KEY(4, 6, KEY_UP),
	KEY(4, 7, KEY_EQUAL),
	KEY(4, 8, KEY_RESERVED),
	KEY(4, 9, KEY_RESERVED),
    KEY(4, 10, KEY_0),
	KEY(4, 11, KEY_RESERVED),
    KEY(4, 12, KEY_Y),
    KEY(4, 13, KEY_S),
    KEY(4, 14, KEY_A),
    KEY(5, 0, KEY_Z),
    KEY(5, 1, KEY_B),
    KEY(5, 2, KEY_V),
    KEY(5, 3, KEY_X),
    KEY(5, 4, KEY_D),
	KEY(5, 5, KEY_RESERVED),
	KEY(5, 6, KEY_RESERVED),
	KEY(5, 7, KEY_RIGHT),
    KEY(5, 8, KEY_RESERVED),
    KEY(5, 9, KEY_RESERVED),
    KEY(5, 10, KEY_RESERVED),
    KEY(5, 11, KEY_RESERVED),
    KEY(5, 12, KEY_N),
    KEY(5, 13, KEY_RESERVED),
	KEY(5, 14, KEY_LEFTSHIFT),
    KEY(6, 0, KEY_RESERVED),
	KEY(6, 1, KEY_RESERVED),
	KEY(6, 2, KEY_RESERVED),
	KEY(6, 3, KEY_RESERVED),
	KEY(6, 4, KEY_RESERVED),
    KEY(6, 5, KEY_COMMA),
    KEY(6, 6, KEY_M),
	KEY(6, 7, KEY_SLASH),
	KEY(6, 8, KEY_SPACE),
    KEY(6, 9, KEY_RIGHTALT),
    KEY(6, 10, KEY_DOT),
	KEY(6, 11, KEY_RESERVED),
    KEY(6, 12, KEY_APOSTROPHE),
    KEY(6, 13, KEY_RESERVED),
    KEY(6, 14, KEY_RESERVED),
};

struct pogo_layerdata {
    u8 row;
    u8 col;
};

struct pogo_keydata {
	struct matrix_keymap_data keymap_data;
	u8 row;
	u8 col;
	u8 layer;
};

static struct pogo_layerdata pogo_layer_data_proto3[] = {
    { /* left opt */
	.row = 1,
	.col = 13,
    },
    { /* right alt/opt */
	.row = 6,
	.col = 1,
    }
};

static struct pogo_keydata pogo_key_data[] =
{
	{
		.keymap_data = {
			.keymap = pogo_keymap_proto1,
			.keymap_size = ARRAY_SIZE(pogo_keymap_proto1),
		},
		.row = 7,
		.col = 15,
		.layer = 1,
	},
	{
		.keymap_data = {
			.keymap = pogo_keymap_proto3,
			.keymap_size = ARRAY_SIZE(pogo_keymap_proto3),
		},
		.row = 7,
		.col = 15,
		.layer = 3,
	},
	{
		.keymap_data = {
			.keymap = pogo_keymap_debug,
			.keymap_size = ARRAY_SIZE(pogo_keymap_debug),
		},
		.row = 7,
		.col = 15,
		.layer = 1,
	},
	{
		.keymap_data = {
			.keymap = pogo_keymap_proto3_type_c,
			.keymap_size = ARRAY_SIZE(pogo_keymap_proto3_type_c),
		},
		.row = 7,
		.col = 15,
		.layer = 1,
	},
};

int pogo_register_uart_keyboard(struct rm_pogo_data *pdata)
{
	struct input_dev *kb_dev;
	const unsigned short *keycodes;
	int cnt, ret;

	if (pdata->dev_info.keylayout >= ARRAY_SIZE(pogo_key_data)) {
		pdata->dev_info.keylayout = 1;
		WARN(1, "Unknown keyboard layout (%d) configured in keyboard! Falling back to version 1.",
		     pdata->dev_info.keylayout);
	}

	kb_dev = input_allocate_device();
	if (!kb_dev)
		goto kb_fail1;

	pdata->kb_dev = kb_dev;
	pdata->kb_row_shift = get_count_order(pogo_key_data[pdata->dev_info.keylayout].col);
	pdata->layer = 0;
	pdata->layers = pogo_key_data[pdata->dev_info.keylayout].layer;
	kb_dev->name = "rM_Keyboard";
	kb_dev->phys = "pogo/input0";
	kb_dev->id.bustype = BUS_HOST;
	kb_dev->id.vendor  = 0x2edd;		/* rM vendor ID */
	kb_dev->id.product = 0x0001;		/* to be decided */
	kb_dev->id.version = 0x0100;
	kb_dev->dev.parent = pdata->dev;

	ret = matrix_keypad_build_keymap(&pogo_key_data[pdata->dev_info.keylayout].keymap_data, NULL,
					   pogo_key_data[pdata->dev_info.keylayout].row *
					   pogo_key_data[pdata->dev_info.keylayout].layer,
					   pogo_key_data[pdata->dev_info.keylayout].col,
					   NULL, kb_dev);
	if (ret) {
		dev_err(pdata->dev, "failed to build keymap\n");
		goto kb_fail1;
	}

	/* setup kb_dev device */
	__set_bit(EV_KEY, kb_dev->evbit);
	__set_bit(EV_REP, kb_dev->evbit);

	ret = input_register_device(kb_dev);
	if (ret)
		goto kb_fail2;

	dev_dbg(pdata->dev, "row_shift: %d, keycodemax %d\n",
		pdata->kb_row_shift, kb_dev->keycodemax);
	keycodes = pdata->kb_dev->keycode;
	for (cnt = 0; cnt < kb_dev->keycodemax; cnt += pogo_key_data[pdata->dev_info.keylayout].col)
		dev_dbg(pdata->dev, "%d: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", cnt,
			keycodes[cnt], keycodes[cnt + 1], keycodes[cnt + 2],
			keycodes[cnt + 3], keycodes[cnt + 4],
			keycodes[cnt + 5], keycodes[cnt + 6],
			keycodes[cnt + 7], keycodes[cnt + 8],
			keycodes[cnt + 9], keycodes[cnt + 10],
			keycodes[cnt + 11], keycodes[cnt + 12],
			keycodes[cnt + 13], keycodes[cnt + 14]);

	return 0;

kb_fail2:
	input_free_device(kb_dev);

kb_fail1:
	return -EINVAL;
}

void pogo_keyboard_report(struct rm_pogo_data *pdata, u8 val)
{
	unsigned int i, col, row, key, from_layer, to_layer, curr_layer;
	bool toggle = false;
	const unsigned short *keycodes = pdata->kb_dev->keycode;

	// determine position
	curr_layer = pdata->layer;
	row = 0x7 & (val >> 1);
	col = 0xf & (val >> 4);

	// handle layer changes
	for(i = 0; i < ARRAY_SIZE(pogo_layer_data_proto3); ++i) {
	    // skip if layer doesn't exist
	    if(i + 1 >= pdata->layers) {
		continue;
	    }
	    // skip if toggle key not pressed
	    if(row != pogo_layer_data_proto3[i].row || col != pogo_layer_data_proto3[i].col) {
		continue;
	    }

	    toggle = true;
	    if(val & 1) {
		from_layer = 0;
		to_layer = i + 1;
	    } else {
		from_layer = i + 1;
		to_layer = 0;
	    }

	    // skip if expected layer doesn't match
	    if(pdata->layer != from_layer) {
		continue;
	    }

	    // apply layer
	    pdata->layer = to_layer;
	    dev_dbg(pdata->dev, "Change layer from %d to %d", from_layer, to_layer);
	    break;
	}

	// ignore if any toggle key pressed
	if(toggle) {
	    return;
	}

	// report key down for pressed key
	if (val & 1) {
	    key = MATRIX_SCAN_CODE(row + (7 * curr_layer), col, pdata->kb_row_shift);
	    input_report_key(pdata->kb_dev, keycodes[key], 1);
	// report key up for all layers
	} else {
	    for(i = 0; i < pdata->layers; ++i) {
		key = MATRIX_SCAN_CODE(row + (7 * i), col, pdata->kb_row_shift);
		input_report_key(pdata->kb_dev, keycodes[key], 0);
	    }
	}
	input_sync(pdata->kb_dev);

	dev_dbg(pdata->dev, "Report row %d col %d key_idx %d code %d active %d\n",
		row, col, key, keycodes[key], val & 1);
}
