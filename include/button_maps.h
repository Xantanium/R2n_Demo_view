#pragma once

#include "../lib/API/Drivers/Controller/Xbox/controller_enums.h"
///////////////////////////////////////////////////////////////////////////////
// BUTTON MAPS
///////////////////////////////////////////////////////////////////////////////

#define B_RELAY ButtonEnum::START
#define B_MPUSE ButtonEnum::R3
#define B_TURRET_MODE ButtonEnum::R1
#define B_INVERT ButtonEnum::BACK
#define B_MBACK_DIR ButtonEnum::L3

// Transfer Mechanism
#define T_TRANSFER_UP JoyEnum::R2
#define T_TRANSFER_DOWN JoyEnum::L2

// Dribbling Mechanism
#define B_CLAW_TOGGLE ButtonEnum::X
#define B_DRIBBLE_AUTO ButtonEnum::B

// Throwing Mechanism
#define B_THROW_SPEED_INC ButtonEnum::R1
#define B_THROW_SPEED_DEC ButtonEnum::L1

// DEFENCE MECHANISM
#define B_DEFENCE_UP ButtonEnum::UP
#define B_DEFENCE_DOWN ButtonEnum::DOWN

// TURRET GUIDEWAY
#define B_VARIABLE_GUIDE_INC ButtonEnum::UP
#define B_VARIABLE_GUIDE_DEC ButtonEnum::DOWN

///////////////////////////////////////////////////////////////////////////////
// STATE ENUMS
///////////////////////////////////////////////////////////////////////////////

#define TRANSFER_UP 1
#define CLAW_OPEN 1
