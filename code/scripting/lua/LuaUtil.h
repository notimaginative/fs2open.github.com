
#ifndef LUA_UTIL_H
#define LUA_UTIL_H
#pragma once

#include "LuaValue.h"
#include "LuaTable.h"
#include "LuaConvert.h"

#include <type_traits> //for std::underlying_type


namespace luacpp {
class LuaTable;

namespace util {

/**
 * @brief Initializes this sublibrary in the specified lua state
 * @param L The lua state to initialize with. Must be the main thread.
 */
void initializeLuaSupportLib(lua_State* L);

/**
 * @brief From the specified state gets the main thread.
 * @param L The state to query
 */
lua_State* getMainThread(lua_State* L);

/**
 * @brief Fills the given container with all key-value pairs.
 * Each key-value pair is converted into the type given to the Container and then push_back is used to add it.
 *
 * @param table The table to be used
 * @param keyValueList The container which should be used
 * @return void
 *
 * @tparam Container Must be a container type with a push_back function and must have a value_type
 * 	which exposes a first_type and second_type typedef (for example std::pair).
 */
template<typename Container>
void tableListPairs(LuaTable& table, Container& keyValueList) {
	typedef typename Container::value_type::first_type key_type;
	typedef typename Container::value_type::second_type value_type;

	keyValueList.clear();
	lua_State* L = table.getLuaState();

	for (auto val : table) {
		val.first.pushValue(L);
		key_type key;
		if (!convert::popValue(L, key)) {
			throw LuaException("Failed to pop key!");
		}

		val.second.pushValue(L);
		value_type value;

		if (!convert::popValue(L, value)) {
			throw LuaException("Failed to pop value!");
		}

		keyValueList.push_back(std::make_pair(key, value));
	}
}

/**
 * @brief Fills the given list with all the values from the table.
 * This will fill the list with the same value as @c ipairs function in lua.
 *
 * @param table The table to be used
 * @param list The list into which the values should be inserted
 * @return void
 *
 * @tparam Container The type of the container into which the values should be inserted,
 * 	must have a value_type typedef so the lua values can be converted and a push_back function.
 *
 * @exception LuaException May be thrown if an element of the table is not convertible to
 * 	Container::value_type.
 */
template<typename Container>
void tableToList(LuaTable& table, Container& list) {
	list.clear();

	size_t length = table.getLength();

	// Lua arrays begin at 1
	for (size_t i = 1; i <= length; ++i) {
		list.push_back(table.getValue<typename Container::value_type>(i));
	}
}

const char* getValueName(ValueType type);

/**
 * Processes a LuaValue and returns a vec3d.  Variants of LuaValue that are understood by this function are:
 * a) a Vector object, e.g. something returned from ba.createVector()
 * b) an array with three elements, e.g. {0.0, 0.0, 0.0}
 * c) a table with x/X, y/Y, and z/Z entries, e.g. {'x': 0.0, 'y': 0.0, 'z': 0.0}
 * 
 * Throws a LuaException if the conversion was not successful.
 */
vec3d valueToVec3d(const LuaValue &luaValue);

}
}

#endif // LUA_UTIL_H
