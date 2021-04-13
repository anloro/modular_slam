/**
 * @file   WorldModel.h
 * @brief  The main class depicting the "world model".
 * @author Ángel Lorente Rogel
 * @date   07/04/2021
 */

#include <string>
#include <memory>
#include <shared_mutex>

#include "Entities.h"

namespace alorrog{
// this lets you define a parameter that could be any datatype 
template <typename T>
// an allocator is the Allocator used in every std container
using FastAllocator = std::allocator<T>;

// with the template we can also define classes
template <class T, class Compare = std::less<T>>
// a set is an associative container, where the elements are unique and 
// sorted. The sorting order can be stablished. In this case is using <
// so it is ascending order x0<x1<x2<...<xn
// as we are using a undefined datatype we have to define the allocator 
// and the comparison method
// set<key, compare, allocator>
using fast_set = std::set<T, Compare, FastAllocator<T>>;

template <class Key, class T, class Compare = std::less<Key>>
using fast_map =
    std::map<Key, T, Compare, FastAllocator<std::pair<const Key, T>>>;

class WorldModelData{
    public:
        struct EntitiesContainer;
        struct FactorsContainer;

        std::string map_name_;

        // Entities
        std::unique_ptr<EntitiesContainer> entities_;
        // entity_connected_factors_t entity_connected_factors_;
        std::shared_mutex entities_mtx_;

        /** All observations, constraints, etc. as generic "factors".
     * Indexed by a unique fid_t; */
        std::unique_ptr<FactorsContainer> factors_;
        std::shared_mutex factors_mtx_;
};

class WorldModel{
    public:
        WorldModel();
    private:

};
}