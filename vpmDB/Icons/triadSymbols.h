/* SPDX-FileCopyrightText: 2023 SAP SE
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file is part of FEDEM - https://openfedem.org
 */
/*!
  \file triadSymbols.h
  \brief Icons for triads of different type.
  \details The triad symbols are used in the Objects view to give various triad
  icons reflecting the DOF status and properties associated with each triad.
  The exclamation mark is used on all triads that are not OK.
*/

const char* exclamation_xpm[] = {
"3 11 2 1",
" 	c None",
".	c #FF4203",
"...",
"...",
"...",
"...",
"...",
"...",
"...",
"   ",
"...",
"...",
"..."};

const char* triad_fixed_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"      ....      ",
"     .    .     ",
"     .    .     ",
"      ....      ",
"       ..       ",
"      .  .      ",
"     .    .     ",
"    .      .    ",
"................",
"  .  .  .  .  . ",
" .  .  .  .  .  ",
".  .  .  .  .   ",
"                ",
"                "};

const char* triad_prescribed_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",	
"   .            ",
"    .   ....    ",
"...... .    .   ",
"    .  .    .   ",
"   .    ....    ",
"         ..     ",
"        .  .    ",
"       .    .   ",
"      .      .  ",
"................",
"                ",
"................",
"  .  .  .  .  . ",
" .  .  .  .  .  ",
".  .  .  .  .   ",
"                "};

const char* triad_load_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"                ",
"                ",
"         ...    ",
"   .    .   .   ",
"    .  .     .  ",
"...... .     .  ",
"    .  .     .  ",
"   .    .   .   ",
"         ...    ",
"                ",
"                ",
"                ",
"                ",
"                "};

const char* triad_mass_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"                ",
"                ",
"       ...      ",
"      .....     ",
"     .......    ",
"     .......    ",
"     .......    ",
"      .....     ",
"       ...      ",
"                ",
"                ",
"                ",
"                ",
"                "};

const char* triad_massLoad_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"                ",
"                ",
"         ...    ",
"   .    .....   ",
"    .  .......  ",
"...... .......  ",
"    .  .......  ",
"   .    .....   ",
"         ...    ",
"                ",
"                ",
"                ",
"                ",
"                "};

const char* triad_master_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"                ",
"                ",
"                ",
"                ",
"   ...      ... ",
"   ....    .... ",
"   .. ..  .. .. ",
"   ..  ....  .. ",
"   ..   ..   .. ",
"   ..        .. ",
"   ..        .. ",
"                ",
"                ",
"                "};

const char* triad_slave_xpm[] = {
"16 16 2 1",
" 	c None",
".	c #000000",
"                ",
"                ",
"                ",
"                ",
"                ",
"                ",
"        ......  ",
"       ..    .  ",
"       ..       ",
"        .....   ",
"            ..  ",
"       .    ..  ",
"       ......   ",
"                ",
"                ",
"                "};
