//
//

#include "base.h"

#include "globalincs/version.h"

#include "freespace.h"

#include "gamesequence/gamesequence.h"
#include "libs/discord/discord.h"
#include "mission/missiontraining.h"
#include "network/multi.h"
#include "parse/parselo.h"
#include "parse/sexp.h"
#include "pilotfile/pilotfile.h"
#include "playerman/player.h"
#include "scripting/api/objs/bytearray.h"
#include "scripting/api/objs/control_info.h"
#include "scripting/api/objs/enums.h"
#include "scripting/api/objs/gameevent.h"
#include "scripting/api/objs/gamestate.h"
#include "scripting/api/objs/player.h"
#include "scripting/api/objs/vecmath.h"
#include "scripting/util/LuaValueDeserializer.h"
#include "scripting/util/LuaValueSerializer.h"
#include "utils/Random.h"
#include "cmdline/cmdline.h"


namespace scripting {
extern SCP_string ade_tostring(lua_State* L, int argnum, bool add_typeinfo);
	
namespace api {
using Random = ::util::Random;

//**********LIBRARY: Base
ADE_LIB(l_Base, "Base", "ba", "Base FreeSpace 2 functions");

ADE_FUNC(print, l_Base, "string Message", "Prints a string", nullptr, nullptr)
{
#ifndef NDEBUG
	auto str = ade_tostring(L, -1, false);
	nprintf(("scripting", "%s", str.c_str()));
#else
	SCP_UNUSED(L);
#endif

	return ADE_RETURN_NIL;
}

ADE_FUNC(println, l_Base, "string Message", "Prints a string with a newline", nullptr, nullptr)
{
#ifndef NDEBUG
	auto str = ade_tostring(L, -1, false);
	nprintf(("scripting", "%s\n", str.c_str()));
#else
	SCP_UNUSED(L);
#endif

	return ADE_RETURN_NIL;
}

ADE_FUNC(warning, l_Base, "string Message", "Displays a FreeSpace warning (debug build-only) message with the string provided", nullptr, nullptr)
{
#ifndef NDEBUG
	auto str = ade_tostring(L, -1, false);

	if (Cmdline_lua_devmode) {
		nprintf(("scripting", "WARNING: %s\n", str.c_str()));
	} else {
		Warning(LOCATION, "%s", str.c_str());
	}
#else
	SCP_UNUSED(L);
	Global_warning_count++;
#endif

	return ADE_RETURN_NIL;
}

ADE_FUNC(error, l_Base, "string Message", "Displays a FreeSpace error message with the string provided", nullptr, nullptr)
{
	auto str = ade_tostring(L, -1, false);

	if (Cmdline_lua_devmode) {
		nprintf(("scripting", "ERROR: %s\n", str.c_str()));
	} else {
		Error(LOCATION, "%s", lua_tostring_nullsafe(L, -1));
	}

	return ADE_RETURN_NIL;
}

ADE_FUNC(rand32,
	l_Base,
	"[number a, number b]",
	"Calls FSO's Random::next() function, which is higher-quality than Lua's ANSI C math.random().  If called with no arguments, returns a random integer from [0, 0x7fffffff].  If called with one argument, returns an integer from [0, a).  If called with two arguments, returns an integer from [a, b].",
	"number",
	"A random integer")
{
	int a, b;
	int numargs = ade_get_args(L, "|ii", &a, &b);

	int result;
	if (numargs == 2) {
		if (a <= b) {
			result = Random::next(a, b);
		} else {
			LuaError(L, "rand32() script function was passed an invalid range (%d ... %d)!", a, b);
			result = a; // match behavior of rand_sexp()
		}
	} else if (numargs == 1) {
		if (a > 0) {
			result = Random::next(a);
		} else {
			LuaError(L, "rand32() script function was passed an invalid modulus (%d)!", a);
			result = 0;
		}
	} else {
		result = Random::next();
	}

	return ade_set_args(L, "i", result);
}

ADE_FUNC(rand32f,
	l_Base,
	"[number max]",
	"Calls FSO's Random::next() function and transforms the result to a float.  If called with no arguments, returns a random float from [0.0, 1.0).  If called with one argument, returns a float from [0.0, max).",
	"number",
	"A random float")
{
	float _max;
	int numargs = ade_get_args(L, "|f", &_max);

	// see also frand()
	int num;
	do {
		num = Random::next();
	} while (num == Random::MAX_VALUE);
	float result = i2fl(num) * Random::INV_F_MAX_VALUE;

	if (numargs > 0)
		result *= _max;

	return ade_set_args(L, "f", result);
}

ADE_FUNC(createOrientation,
	l_Base,
	ade_overload_list({nullptr,
		"number p, number b, number h",
		"number r1c1, number r1c2, number r1c3, number r2c1, number r2c2, number r2c3, number r3c1, number r3c2, "
		"number r3c3"}),
	"Given 0 arguments, creates an identity orientation; 3 arguments, creates an orientation from pitch/bank/heading (in radians); 9 arguments, creates an orientation from a 3x3 row-major order matrix.",
	"orientation",
	"New orientation object, or the identity orientation on failure")
{
	matrix m;
	int numargs = ade_get_args(L, "|fffffffff", &m.a1d[0], &m.a1d[1], &m.a1d[2], &m.a1d[3], &m.a1d[4], &m.a1d[5], &m.a1d[6], &m.a1d[7], &m.a1d[8]);
	if(!numargs)
	{
		return ade_set_args(L, "o", l_Matrix.Set( matrix_h(&vmd_identity_matrix) ));
	}
	else if(numargs == 3)
	{
		angles a = {m.a1d[0], m.a1d[1], m.a1d[2]};
		return ade_set_args(L, "o", l_Matrix.Set(matrix_h(&a)));
	}
	else if(numargs == 9)
	{
		return ade_set_args(L, "o", l_Matrix.Set(matrix_h(&m)));
	}

	return ade_set_error(L, "o", l_Matrix.Set(matrix_h()));
}

ADE_FUNC(createOrientationFromVectors, l_Base, "[vector fvec, vector uvec, vector rvec]",
	"Given 0 to 3 arguments, creates an orientation object from 0 to 3 vectors.  (This is essentially a wrapper for the vm_vector_2_matrix function.)  If supplied 0 arguments, this will return the identity orientation.  The first vector, if supplied, must be non-null.",
	"orientation",
	"New orientation object, or the identity orientation on failure")
{
	vec3d *fvec = nullptr, *uvec = nullptr, *rvec = nullptr;
	int numargs = ade_get_args(L, "|ooo", l_Vector.GetPtr(&fvec), l_Vector.GetPtr(&uvec), l_Vector.GetPtr(&rvec));
	if (!numargs)
	{
		return ade_set_args(L, "o", l_Matrix.Set(matrix_h(&vmd_identity_matrix)));
	}
	else
	{
		// if we have any vectors, the first one should be non-null
		if (fvec == nullptr)
			return ade_set_error(L, "o", l_Matrix.Set(matrix_h()));

		matrix m;
		vm_vector_2_matrix(&m, fvec, uvec, rvec);
		return ade_set_args(L, "o", l_Matrix.Set(matrix_h(&m)));
	}
}

ADE_FUNC(createVector, l_Base, "[number x, number y, number z]", "Creates a vector object", "vector", "Vector object")
{
	vec3d v3 = vmd_zero_vector;
	ade_get_args(L, "|fff", &v3.xyz.x, &v3.xyz.y, &v3.xyz.z);

	return ade_set_args(L, "o", l_Vector.Set(v3));
}

ADE_FUNC(createRandomVector, l_Base, nullptr, "Creates a random normalized vector object.", "vector", "Vector object")
{
	vec3d v3;
	vm_vec_random_in_sphere(&v3, &vmd_zero_vector, 1.0f, true);
	return ade_set_args(L, "o", l_Vector.Set(v3));
}

ADE_FUNC(createRandomOrientation, l_Base, nullptr, "Creates a random orientation object.", "orientation", "Orientation object")
{
	vec3d fvec, uvec;
	matrix fvec_orient, final_orient;

	vm_vec_random_in_sphere(&fvec, &vmd_zero_vector, 1.0f, true);
	vm_vector_2_matrix_norm(&fvec_orient, &fvec, nullptr, nullptr);

	vm_vec_random_in_circle(&uvec, &vmd_zero_vector, &fvec_orient, 1.0f, true);

	vm_vector_2_matrix_norm(&final_orient, &fvec, &uvec);

	return ade_set_args(L, "o", l_Matrix.Set(matrix_h(&final_orient)));
}

ADE_FUNC(createSurfaceNormal,
	l_Base,
	"vector point1, vector point2, vector point3",
	"Determines the surface normal of the plane defined by three points.  Returns a normalized vector.",
	"vector",
	"The surface normal, or NIL if a handle is invalid")
{
	vec3d *p0 = nullptr, *p1 = nullptr, *p2 = nullptr;
	if (!ade_get_args(L, "ooo", l_Vector.GetPtr(&p0), l_Vector.GetPtr(&p1), l_Vector.GetPtr(&p2)))
		return ADE_RETURN_NIL;

	vec3d dest;
	vm_vec_normal(&dest, p0, p1, p2);
	return ade_set_args(L, "o", l_Vector.Set(dest));
}

ADE_FUNC(findIntersection,
	l_Base,
	"vector line1_point1, vector line1_point2, vector line2_point1, vector line2_point2",
	"Determines the point at which two lines intersect.  (The lines are assumed to extend infinitely in both directions; the intersection will not necessarily be between the points.)",
	"vector, number",
	"Returns two arguments.  The first is the point of intersection, if it exists and is unique (otherwise it will be NIL).  The second is the find_intersection return value: 0 for a unique intersection, -1 if the lines are colinear, and -2 if the lines do not intersect.")
{
	vec3d *p0 = nullptr, *p0_end = nullptr, *p1 = nullptr, *p1_end = nullptr;
	if (!ade_get_args(L, "oooo", l_Vector.GetPtr(&p0), l_Vector.GetPtr(&p0_end), l_Vector.GetPtr(&p1), l_Vector.GetPtr(&p1_end)))
		return ADE_RETURN_NIL;

	// note: we must translate from this API's two-points method to the code's API of a reference point and a direction vector
	vec3d v0, v1;
	vm_vec_sub(&v0, p0_end, p0);
	vm_vec_sub(&v1, p1_end, p1);

	float scalar;
	int retval = find_intersection(&scalar, p0, p1, &v0, &v1);

	if (retval == 0)
	{
		// per comments:
		// If you want the coords of the intersection, scale v0 by s, then add p0.
		vm_vec_scale(&v0, scalar);
		vm_vec_add2(&v0, p0);

		return ade_set_args(L, "oi", l_Vector.Set(v0), retval);
	}
	else
		return ade_set_args(L, "*i", retval);
}

ADE_FUNC(findPointOnLineNearestSkewLine,
	l_Base,
	"vector line1_point1, vector line1_point2, vector line2_point1, vector line2_point2",
	"Determines the point on line 1 closest to line 2 when the lines are skew (non-intersecting in 3D space).  (The lines are assumed to extend infinitely in both directions; the point will not necessarily be between the other points.)",
	"vector",
	"The closest point, or NIL if a handle is invalid")
{
	vec3d *p0 = nullptr, *p0_end = nullptr, *p1 = nullptr, *p1_end = nullptr;
	if (!ade_get_args(L, "oooo", l_Vector.GetPtr(&p0), l_Vector.GetPtr(&p0_end), l_Vector.GetPtr(&p1), l_Vector.GetPtr(&p1_end)))
		return ADE_RETURN_NIL;

	// note: we must translate from this API's two-points method to the code's API of a reference point and a direction vector
	vec3d v0, v1;
	vm_vec_sub(&v0, p0_end, p0);
	vm_vec_sub(&v1, p1_end, p1);

	vec3d dest;
	find_point_on_line_nearest_skew_line(&dest, p0, &v0, p1, &v1);
	return ade_set_args(L, "o", l_Vector.Set(dest));
}

ADE_FUNC(getFrametimeOverall, l_Base, nullptr, "The overall frame time in fix units (seconds * 65536) since the engine has started", "number", "Overall time (fix units)")
{
	return ade_set_args(L, "x", game_get_overall_frametime());
}

ADE_FUNC(getSecondsOverall, l_Base, nullptr, "The overall time in seconds since the engine has started", "number", "Overall time (seconds)")
{
	return ade_set_args(L, "f", f2fl(game_get_overall_frametime()));
}

ADE_FUNC(getMissionFrametime, l_Base, nullptr, "Gets how long this frame is calculated to take. Use it to for animations, physics, etc to make incremental changes. Increased or decreased based on current time compression", "number", "Frame time (seconds)")
{
	return ade_set_args(L, "f", flFrametime);
}

ADE_FUNC(getRealFrametime, l_Base, nullptr, "Gets how long this frame is calculated to take in real time. Not affected by time compression.", "number", "Frame time (seconds)")
{
	return ade_set_args(L, "f", flRealframetime);
}

ADE_FUNC_DEPRECATED(getFrametime, l_Base, 
	"[boolean adjustForTimeCompression]", 
	"Gets how long this frame is calculated to take. Use it to for animations, physics, etc to make incremental changes.", 
	"number", "Frame time (seconds)", 
	gameversion::version(20, 2, 0, 0),
	"The parameter of this function is inverted from the naming (passing true returns non-adjusted time). Please use either getMissionFrametime() or getRealFrametime().")
{
	bool b=false;
	ade_get_args(L, "|b", &b);

	return ade_set_args(L, "f", b ? flRealframetime : flFrametime);
}

ADE_FUNC(getCurrentGameState, l_Base, "[number depth]", "Gets current FreeSpace state; if a depth is specified, the state at that depth is returned. (IE at the in-game options game, a depth of 1 would give you the game state, while the function defaults to 0, which would be the options screen.", "gamestate", "Current game state at specified depth, or invalid handle if no game state is active yet")
{
	int depth = 0;
	ade_get_args(L, "|i", &depth);

	if(depth > gameseq_get_depth())
		return ade_set_args(L, "o", l_GameState.Set(gamestate_h()));

	return ade_set_args(L, "o", l_GameState.Set(gamestate_h(gameseq_get_state(depth))));
}

ADE_FUNC(getCurrentMPStatus, l_Base, nullptr, "Gets this computers current MP status", "string", "Current MP status" )
{
	if ( MULTIPLAYER_MASTER )
		return ade_set_args(L, "s", "MULTIPLAYER_MASTER");

	if ( MULTIPLAYER_HOST )
		return ade_set_args(L, "s", "MULTIPLAYER_HOST");

	if ( MULTIPLAYER_CLIENT )
		return ade_set_args(L, "s", "MULTIPLAYER_CLIENT");

	if ( MULTIPLAYER_STANDALONE )
		return ade_set_args(L, "s", "MULTIPLAYER_STANDALONE");

	return ade_set_args(L, "s", "SINGLEPLAYER");
}

ADE_FUNC(getCurrentPlayer, l_Base, NULL, "Gets a handle of the currently used player.<br><b>Note:</b> If there is no current player then the first player will be returned, check the game state to make sure you have a valid player handle.", "player", "Player handle")
{
	return ade_set_args(L, "o", l_Player.Set(player_h(&Players[Player_num])));
}

ADE_FUNC(loadPlayer, l_Base, "string callsign", "Loads the player with the specified callsign.", "player",
         "Player handle or invalid handle on load failure")
{
	const char* callsign;
	if (!ade_get_args(L, "s", &callsign)) {
		return ade_set_error(L, "o", l_Player.Set(player_h()));
	}

	player plr;
	plr.reset();
	pilotfile loader;
	if (!loader.load_player(callsign, &plr)) {
		return ade_set_error(L, "o", l_Player.Set(player_h()));
	}

	return ade_set_args(L, "o", l_Player.Set(player_h(plr)));
}

ADE_FUNC(savePlayer, l_Base, "player plr", "Saves the specified player.", "boolean",
         "true of successful, false otherwise")
{
	player_h* plh;
	if (!ade_get_args(L, "o", l_Player.GetPtr(&plh))) {
		return ADE_RETURN_FALSE;
	}

	pilotfile loader;
	return ade_set_args(L, "b", loader.save_player(plh->get()));
}

ADE_FUNC(setControlMode,
	l_Base,
	"nil|enumeration mode /* LE_*_CONTROL */",
	"Sets the current control mode for the game.",
	"string",
	"Current control mode")
{
	enum_h *e = NULL;
	if (!(ade_get_args(L, "|o", l_Enum.GetPtr(&e)))) {
		if (lua_game_control & LGC_NORMAL)
			return ade_set_args(L, "s", "NORMAL");
		else if (lua_game_control & LGC_STEERING)
			return ade_set_args(L, "s", "STEERING");
		else if (lua_game_control & LGC_FULL)
			return ade_set_args(L, "s", "FULL");
		else
			return ade_set_error(L, "s", "");
	}

	if (!e) {
		return ade_set_error(L, "s", "");
	}

	switch (e->index) {
		case LE_NORMAL_CONTROLS:
			lua_game_control |= LGC_NORMAL;
			lua_game_control &= ~(LGC_STEERING|LGC_FULL);
			return ade_set_args(L, "s", "NORMAL CONTROLS");
		case LE_LUA_STEERING_CONTROLS:
			lua_game_control |= LGC_STEERING;
			lua_game_control &= ~(LGC_NORMAL|LGC_FULL);
			return ade_set_args(L, "s", "LUA STEERING CONTROLS");
		case LE_LUA_FULL_CONTROLS:
			lua_game_control |= LGC_FULL;
			lua_game_control &= ~(LGC_STEERING|LGC_NORMAL);
			return ade_set_args(L, "s", "LUA FULL CONTROLS");
		default:
			return ade_set_error(L, "s", "");
	}
}

ADE_FUNC(setButtonControlMode,
	l_Base,
	"nil|enumeration mode /* LE_*_BUTTON_CONTROL */",
	"Sets the current control mode for the game.",
	"string",
	"Current control mode")
{
	enum_h *e = NULL;
	if (!(ade_get_args(L, "|o", l_Enum.GetPtr(&e)))) {
		if (lua_game_control & LGC_B_NORMAL)
			return ade_set_args(L, "s", "NORMAL");
		else if (lua_game_control & LGC_B_OVERRIDE)
			return ade_set_args(L, "s", "OVERRIDE");
		else if (lua_game_control & LGC_B_ADDITIVE)
			return ade_set_args(L, "s", "ADDITIVE");
		else
			return ade_set_error(L, "s", "");
	}

	if (!e) {
		return ade_set_error(L, "s", "");
	}

	switch (e->index) {
		case LE_NORMAL_BUTTON_CONTROLS:
			lua_game_control |= LGC_B_NORMAL;
			lua_game_control &= ~(LGC_B_ADDITIVE|LGC_B_OVERRIDE);
			return ade_set_args(L, "s", "NORMAL BUTTON CONTROL");
		case LE_LUA_ADDITIVE_BUTTON_CONTROL:
			lua_game_control |= LGC_B_ADDITIVE;
			lua_game_control &= ~(LGC_B_NORMAL|LGC_B_OVERRIDE);
			return ade_set_args(L, "s", "LUA ADDITIVE BUTTON CONTROL");
		case LE_LUA_OVERRIDE_BUTTON_CONTROL:
			lua_game_control |= LGC_B_OVERRIDE;
			lua_game_control &= ~(LGC_B_ADDITIVE|LGC_B_NORMAL);
			return ade_set_args(L, "s", "LUA OVERRIDE BUTTON CONTROL");
		default:
			return ade_set_error(L, "s", "");
	}
}

ADE_FUNC(getControlInfo, l_Base, nullptr, "Gets the control info handle.", "control_info", "control info handle")
{
	return ade_set_args(L, "o", l_Control_Info.Set(1));
}

ADE_FUNC(setTips, l_Base, "boolean", "Sets whether to display tips of the day the next time the current pilot enters the mainhall.", nullptr, nullptr)
{
	if (Player == NULL)
		return ADE_RETURN_NIL;

	bool tips = false;

	if (!ade_get_args(L, "b", &tips))
		return ADE_RETURN_NIL;

	if (tips)
		Player->tips = 1;
	else
		Player->tips = 0;

	return ADE_RETURN_NIL;
}

ADE_FUNC(getGameDifficulty, l_Base, nullptr,
         "Returns the difficulty level from 1-5, 1 being the lowest, (Very Easy) and 5 being the highest (Insane)",
         "number", "Difficulty level as integer")
{
	return ade_set_args(L, "i", Game_skill_level+1);
}

ADE_FUNC(postGameEvent, l_Base, "gameevent Event", "Sets current game event. Note that you can crash FreeSpace 2 by posting an event at an improper time, so test extensively if you use it.", "boolean", "True if event was posted, false if passed event was invalid")
{
	gameevent_h *gh = NULL;
	if(!ade_get_args(L, "o", l_GameEvent.GetPtr(&gh)))
		return ade_set_error(L, "b", false);

	if(!gh->isValid())
		return ade_set_error(L, "b", false);

	gameseq_post_event(gh->Get());

	return ADE_RETURN_TRUE;
}

ADE_FUNC(XSTR,
	l_Base,
	"string text, number id, boolean tstrings=true",
	"Gets the translated version of text with the given id. "
	"This uses the tstrings.tbl for performing the translation by default. Set tstrings to false to use "
	"strings.tbl instead. Passing -1 as the id will always return the given text.",
	"string",
	"The translated text")
{
	const char* text = nullptr;
	int id = -1;
	bool use_tstrings = true;

	if (!ade_get_args(L, "si|b", &text, &id, &use_tstrings)) {
		return ADE_RETURN_NIL;
	}

	SCP_string translated;
	if (use_tstrings) {
		SCP_string xstr;
		sprintf(xstr, "XSTR(\"%s\", %d)", text, id);
		lcl_ext_localize(xstr, translated);
	} else {
		translated = XSTR(text, id);
	}

	return ade_set_args(L, "s", translated.c_str());
}

ADE_FUNC(replaceTokens, 
	l_Base, 
	"string text", 
	"Returns a string that replaces any default control binding to current binding (same as Directive Text). Default binding must be encapsulated by '$$' for replacement to work.",
	"string",
	"Updated string or nil if invalid")
{
	const char* untranslated_str;
	if (!ade_get_args(L, "s", &untranslated_str)) {
		return ADE_RETURN_NIL;
	}

	SCP_string translated_str = message_translate_tokens(untranslated_str);

	return ade_set_args(L, "s", translated_str.c_str());
}

ADE_FUNC(replaceVariables,
	l_Base,
	"string text",
	"Returns a string that replaces any variable name with the variable value (same as text in Briefings, Debriefings, or Messages). Variable name must be preceded by '$' for replacement to work.",
	"string",
	"Updated string or nil if invalid")
{
	const char* untranslated_str;
	if (!ade_get_args(L, "s", &untranslated_str)) {
		return ADE_RETURN_NIL;
	}

	SCP_string translated_str = untranslated_str;
	sexp_replace_variable_names_with_values(translated_str);

	return ade_set_args(L, "s", translated_str.c_str());
}

ADE_FUNC(inMissionEditor, l_Base, nullptr, "Determine if the current script is running in the mission editor (e.g. FRED2). This should be used to control which code paths will be executed even if running in the editor.", "boolean", "true when we are in the mission editor, false otherwise") {
	return ade_set_args(L, "b", Fred_running != 0);
}

ADE_FUNC(inDebug, l_Base, nullptr, "Determines if FSO is running in Release or Debug", "boolean", "true if debug, false if release") {
	#ifndef NDEBUG
		return ADE_RETURN_TRUE;
	#else
		return ADE_RETURN_FALSE;
	#endif
}

ADE_FUNC(isEngineVersionAtLeast,
		 l_Base,
		 "number major, number minor, number build, [number revision = 0]",
		 "Checks if the current version of the engine is at least the specified version. This can be used to check if a feature introduced in a later version of the engine is available.",
		 "boolean",
		 "true if the version is at least the specified version. false otherwise.") {
	int major = 0;
	int minor = 0;
	int build = 0;
	int revision = 0;

	if (!ade_get_args(L, "iii|i", &major, &minor, &build, &revision)) {
		return ade_set_error(L, "b", false);
	}

	auto version = gameversion::version(major, minor, build, revision);

	return ade_set_args(L, "b", gameversion::check_at_least(version));
}

ADE_FUNC(usesInvalidInsteadOfNil,
	l_Base,
	nullptr,
	"Checks if the '$Lua API returns nil instead of invalid object:' option is set in game_settings.tbl.",
	"boolean",
	"true if the option is set, false otherwise")
{
	return Lua_API_returns_nil_instead_of_invalid_object ? ADE_RETURN_TRUE : ADE_RETURN_FALSE;
}

ADE_FUNC(getCurrentLanguage,
		 l_Base,
		 nullptr,
		 "Determines the language that is being used by the engine. This returns the full name of the language (e.g. \"English\").",
		 "string",
		 "The current game language")
{
	const char *lang_name;
	if (Lcl_current_lang == LCL_UNTRANSLATED)
		lang_name = "UNTRANSLATED";
	else if (Lcl_current_lang == LCL_RETAIL_HYBRID)
		lang_name = "RETAIL HYBRID";
	else
		lang_name = Lcl_languages[lcl_get_current_lang_index()].lang_name;

	return ade_set_args(L, "s", lang_name);
}

ADE_FUNC(getCurrentLanguageExtension,
		 l_Base,
		 nullptr,
		 "Determines the file extension of the language that is being used by the engine. "
			 "This returns a short code for the current language that can be used for creating language specific file names (e.g. \"gr\" when the current language is German). "
			 "This will return an empty string for the default language.",
		 "string",
		 "The current game language")
{
	int lang = lcl_get_current_lang_index();
	return ade_set_args(L, "s", Lcl_languages[lang].lang_ext);
}

ADE_FUNC(getVersionString, l_Base, nullptr,
         "Returns a string describing the version of the build that is currently running. This is mostly intended to "
         "be displayed to the user and not processed by a script so don't rely on the exact format of the string.",
         "string", "The version information")
{
	auto str = gameversion::get_version_string();
	return ade_set_args(L, "s", str.c_str());
}

ADE_FUNC(getModRootName, l_Base, nullptr,
	"Returns the name of the current mod's root folder.", "string", "The mod root or empty string if the mod runs without a -mod line")
{
	const char* mod = Cmdline_mod;
	if (mod == nullptr) {
		mod = "";
	}
	
	SCP_string str = mod;

	// Trim any trailing folders so we get just the name of the root mod folder
	str = str.substr(0, str.find_first_of(DIR_SEPARATOR_CHAR));

	return ade_set_args(L, "s", str.c_str());
}

ADE_FUNC(getModTitle, l_Base, nullptr,
         "Returns the title of the current mod as defined in game_settings.tbl. Will return an empty string if not defined.",
         "string", "The mod title")
{
	return ade_set_args(L, "s", Mod_title.c_str());
}

ADE_FUNC(getModVersion, l_Base, nullptr,
         "Returns the version of the current mod as defined in game_settings.tbl. If the version is semantic versioning then the "
		 "returned numbers will reflect that. String always returns the complete string. If semantic version is not used then "
		 "the returned numbers will all be -1",
         "string, number, number, number", "The mod version string; the major, minor, patch version numbers or -1 if invalid")
{
	gameversion::version version(Mod_version);

	return ade_set_args(L, "siii", Mod_version.c_str(), version.major, version.minor, version.build);
}

ADE_VIRTVAR(MultiplayerMode, l_Base, "boolean", "Determines if the game is currently in single- or multiplayer mode",
            "boolean",
            "true if in multiplayer mode, false if in singleplayer. If neither is the case (e.g. on game init) nil "
            "will be returned")
{
	bool b;
	if (!ade_get_args(L, "*|b", &b)) {
		return ADE_RETURN_NIL;
	}

	if (ADE_SETTING_VAR) {
		if (b) {
			Game_mode &= ~GM_NORMAL;
			Game_mode |= GM_MULTIPLAYER;
		} else {
			Game_mode &= ~GM_MULTIPLAYER;
			Game_mode |= GM_NORMAL;
		}
	}

	if (Game_mode & GM_MULTIPLAYER) {
		return ADE_RETURN_TRUE;
	} else if (Game_mode & GM_NORMAL) {
		return ADE_RETURN_FALSE;
	} else {
		return ADE_RETURN_NIL;
	}
}

ADE_FUNC(serializeValue,
	l_Base,
	"any value",
	"Serializes the specified value so that it can be stored and restored consistently later. The actual format of the "
	"returned data is implementation specific but will be deserializable by at least this engine version and following "
	"versions.",
	"bytearray",
	"The serialized representation of the value or nil on error.")
{
	luacpp::LuaValue value;
	if (!ade_get_args(L, "a", &value)) {
		return ADE_RETURN_NIL;
	}

	try {
		util::LuaValueSerializer serializer(std::move(value));
		auto serialized = serializer.serialize();

		return ade_set_args(L, "o", l_Bytearray.Set(bytearray_h(std::move(serialized))));
	} catch (const std::exception& e) {
		LuaError(L, "Failed to serialize value: %s", e.what());
		return ADE_RETURN_NIL;
	}
}

ADE_FUNC(deserializeValue,
	l_Base,
	"bytearray serialized",
	"Deserializes a previously serialized Lua value.",
	"any",
	"The deserialized Lua value.")
{
	bytearray_h* array = nullptr;
	if (!ade_get_args(L, "o", l_Bytearray.GetPtr(&array))) {
		return ade_set_args(L, "o", l_Bytearray.Set(bytearray_h()));
	}

	try {
		util::LuaValueDeserializer deserializer(L);
		auto deserialized = deserializer.deserialize(array->data());

		return ade_set_args(L, "a", deserialized);
	} catch (const std::exception& e) {
		LuaError(L, "Failed to deserialize value: %s", e.what());
		return ADE_RETURN_NIL;
	}
}


ADE_FUNC(setDiscordPresence,
	l_Base,
	"string DisplayText, [boolean Gameplay]",
	"Sets the Discord presence to a specific string. If Gameplay is true then the string is ignored and presence will "
	"be set as if the player is in-mission. The latter will fail if the player is not in a mission.",
	nullptr,
	"nothing")
{
	const char* text;
	bool gp = false;
	if (!ade_get_args(L, "s|b", &text, &gp)) {
		return ADE_RETURN_NIL;
	}

	if (gp) {
		if ((Game_mode & GM_IN_MISSION) != 0){
			libs::discord::set_presence_gameplay();
		}
	} else {
		libs::discord::set_presence_string(text);
	}

	return ADE_RETURN_NIL;
}

ADE_FUNC(hasFocus, l_Base, nullptr, "Returns if the game engine has focus or not", "boolean", "True if the game has focus, false if it has been lost")
{
	return ade_set_args(L, "b", os_foreground());

}

//**********SUBLIBRARY: Base/Events
ADE_LIB_DERIV(l_Base_Events, "GameEvents", NULL, "Freespace 2 game events", l_Base);

ADE_INDEXER(l_Base_Events, "number/string IndexOrName", "Array of game events", "gameevent", "Game event, or invalid gameevent handle if index is invalid")
{
	const char* name;
	if(!ade_get_args(L, "*s", &name))
		return ade_set_error(L, "o", l_GameEvent.Set(gameevent_h()));

	int idx = gameseq_get_event_idx(name);

	if(idx < 0)
	{
		idx = atoi(name);

		//Lua-->FS2
		idx--;

		if(idx < 0 || idx >= Num_gs_event_text)
			return ade_set_error(L, "o", l_GameEvent.Set(gameevent_h()));
	}

	return ade_set_args(L, "o", l_GameEvent.Set(gameevent_h(idx)));
}

ADE_FUNC(__len, l_Base_Events, NULL, "Number of events", "number", "Number of events")
{
	return ade_set_args(L, "i", Num_gs_event_text);
}

//**********SUBLIBRARY: Base/States
ADE_LIB_DERIV(l_Base_States, "GameStates", NULL, "Freespace 2 states", l_Base);

ADE_INDEXER(l_Base_States, "number/string IndexOrName", "Array of game states", "gamestate", "Game state, or invalid gamestate handle if index is invalid")
{
	const char* name;
	if(!ade_get_args(L, "*s", &name))
		return ade_set_error(L, "o", l_GameState.Set(gamestate_h()));

	int idx = gameseq_get_state_idx(name);

	if(idx < 0)
	{
		idx = atoi(name);

		//Lua-->FS2
		idx--;

		if(idx < 0 || idx >= Num_gs_state_text)
			return ade_set_error(L, "o", l_GameState.Set(gamestate_h()));
	}

	return ade_set_args(L, "o", l_GameState.Set(gamestate_h(idx)));
}

ADE_FUNC(__len, l_Base_States, NULL, "Number of states", "number", "Number of states")
{
	return ade_set_args(L, "i", Num_gs_state_text);
}


}
}

