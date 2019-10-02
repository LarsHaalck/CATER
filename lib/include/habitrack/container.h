#ifndef PROXY_CONTAINER_H
#define PROXY_CONTAINER_H

#include <cstddef>
#include <vector>

// base class for different types of container proxies
namespace ht
{
template<typename K, typename T>
class ProxyContainer
{
public:
    /* virtual const T& operator[](const K& key) const = 0; */
    /* virtual T& operator[](const K& key) = 0; */

    /* virtual const T& at(const K& key) const = 0; */
    /* virtual T& at(const K& key) = 0; */

    /* // default implementations do nothing */
    virtual void prefetch(std::size_t numElements) = 0;
    virtual void prefetch(const std::vector<K>& elems) = 0;
    virtual void clearCache() = 0;

    virtual ~ProxyContainer() { }
};
} // namespace ht

#endif // PROXY_CONTAINER_H
