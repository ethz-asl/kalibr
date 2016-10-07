/*
 * NodeDistributedCache.hpp
 *
 *  Created on: Oct 4, 2012
 *      Author: hannes
 */

#ifndef NODEDISTRIBUTEDCACHE_HPP_
#define NODEDISTRIBUTEDCACHE_HPP_

#include <memory>
#include <vector>
#include <sm/assert_macros.hpp>
#include <Eigen/Core>

namespace nodecache {

template<typename TNode>
struct NodeCacheAccessor;

template<typename TNode, typename TValue>
class CacheInvalidator{
	virtual ~CacheInvalidator() = 0;
	void virtual invalidate(TNode & node, NodeCacheAccessor<TNode> & nodeCacheAccessor) = 0;
};


template <typename TNode>
class NodeDistributedCache{
public:
	template <typename TValue>
	class NodeCacheSlot;

	class PerNodeCache{
		class PerNodeCacheEntry{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			void setValid(bool valid);
			bool isValid();
		};

	public:
		template <typename TValue> friend class PerNodeCacheValue;
		template <typename TValue> friend class NodeCacheSlot;

		template <typename TValue>
		class PerNodeCacheValue : public PerNodeCacheEntry {
			TValue _value;
			bool _valid;
			PerNodeCacheValue() : _valid(false){
			}
			friend class NodeCacheSlot<TValue>;
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			inline TValue & accessValue(){
				return _value;
			}

			template <typename TFunctor>
			inline const TValue & get(TFunctor & updater){
				if(!isValid()){
					updater(_value);
					setValid(true);
				}
				return _value;
			}

			inline operator TValue & (){
				return _value;
			}

			inline void setValid(bool valid){ _valid = valid; }
			inline bool isValid(){ return _valid; }
		};


		void handleUpdatedValueEvent(){
			for(typename std::vector<PerNodeCacheEntry *>::iterator i = entries.begin(), end = entries.end(); i != end; i++){

			}
		}

		~PerNodeCache(){
			for(typename std::vector<PerNodeCacheEntry *>::iterator i = entries.begin(), end = entries.end(); i != end; i++){
				if(*i!=NULL)
					delete *i;
			}
		}
	private:
		std::vector<PerNodeCacheEntry *> entries;

		PerNodeCacheEntry * & getEntry(size_t index){
			if(index >= entries.size()){
				entries.resize(index+1);
			}
			return entries[index];
		}
	};

	template <typename TValue>
	class NodeCacheSlot {
		size_t _index;
		NodeDistributedCache & _cache;

		NodeCacheSlot(size_t index, NodeDistributedCache & cache) : _index(index), _cache(cache) {}

		friend class NodeDistributedCache<TNode>;

	public:
		~NodeCacheSlot(){
			_cache.freeIndex(_index);
		}
		typedef typename PerNodeCache::template PerNodeCacheValue<TValue> CacheValueT;
		inline TValue & accessValue(TNode & node){
			return accessNodeCache(node).accessValue();
		}

		inline CacheValueT & accessNodeCache(TNode & node){
			typename PerNodeCache::PerNodeCacheEntry * & pEntry = NodeCacheAccessor<TNode>::accessNodeCache(node).getEntry(_index);
			if(pEntry == NULL){
				pEntry = new CacheValueT();
			}
			return *static_cast<CacheValueT *>(pEntry);
		}
	};

	template <typename TValue>
	inline std::shared_ptr<NodeCacheSlot<TValue> > registerCacheableValue(){
		return std::shared_ptr<NodeCacheSlot<TValue> >(new NodeCacheSlot<TValue>(getNextFreeIndex(), *this));
	}
private:
	std::vector<bool> slotUsage;
	int getNextFreeIndex(){
		//TODO optimize : optimize for continuous usage e.g. by introducing a full up to index
		int j = 0;
		for(std::vector<bool>::iterator i = slotUsage.begin(), end = slotUsage.end(); i != end; (i++, j++)){
			if(!*i){
				*i = true;
				return j;
			}
		}
		slotUsage.push_back(true);
		return slotUsage.size() - 1;
	};

	void freeIndex(int i){
		SM_ASSERT_TRUE_DBG(std::runtime_error, slotUsage[i], "bug in NodeCacheAccessor!");
		slotUsage[i] = false;
	}
public:
	~NodeDistributedCache(){
		for(std::vector<bool>::iterator i = slotUsage.begin(), end = slotUsage.end(); i != end; i++){
			SM_ASSERT_FALSE(std::runtime_error, *i, "memory leak : there are still some slots alive when deleting the entire cache!");
		}
	}
};

template<typename TNode>
struct NodeCacheAccessor {
	static inline typename NodeDistributedCache<TNode>::PerNodeCache & accessNodeCache(TNode & node){ return node.accessCache();}
};

}

#endif /* NODEDISTRIBUTEDCACHE_HPP_ */
