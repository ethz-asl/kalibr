#ifndef DYNAMICORTEMPLATEINT_HPP_
#define DYNAMICORTEMPLATEINT_HPP_
#include <sm/assert_macros.hpp>
#include <Eigen/Core>

namespace eigenTools {
	struct DynamicOrTemplateIntBase {
		virtual inline int getValue() const { return -2; };
		virtual ~DynamicOrTemplateIntBase(){};

		inline static bool isDynamic() {
			return false;
		}
	};

	template <int VALUE_T>
	struct DynamicOrTemplateInt : public DynamicOrTemplateIntBase {
		enum { VALUE = VALUE_T, IS_DYNAMIC = 0 };

		DynamicOrTemplateInt(int value = VALUE_T){
			SM_ASSERT_EQ(std::runtime_error, value, (int)VALUE, "Please don't set dynamic parameter value with fixed size template argument to something else!")
		}

		virtual inline int getValue() const {
			return VALUE;
		}

		inline operator int() const {
			return getValue();
		}
	};

	template <>
	struct DynamicOrTemplateInt<Eigen::Dynamic>{
		enum { VALUE = Eigen::Dynamic, IS_DYNAMIC = 1 };
		int _value;

		DynamicOrTemplateInt(int value) : _value(value){
			if(value == Eigen::Dynamic)
				SM_ASSERT_NE(std::runtime_error, value, Eigen::Dynamic, "Please set the dynamic parameter in the constructor when you use the Eigen::Dynamic template argument!")
		}

		inline int getValue() const {
			return _value;
		}

		inline bool isDynamic() const {
			return true;
		}

		inline operator int() const {
			return getValue();
		}
	};

	#define multiplyEigenSize(a, b) (a == Eigen::Dynamic || b == Eigen::Dynamic? Eigen ::Dynamic : a * b)

	template<int A, int B>
	inline DynamicOrTemplateInt <multiplyEigenSize(A, B) > operator * (const DynamicOrTemplateInt<A> & a, const DynamicOrTemplateInt<B> & b){
		return DynamicOrTemplateInt<multiplyEigenSize(A, B)>(multiplyEigenSize(a.getValue(), b.getValue()));
	}

	template <typename T, int ISize> struct DynOrStaticSizedArray {
		T p[ISize];
		DynOrStaticSizedArray(int size){
			if(size != ISize) throw new std::runtime_error("dynamic and static size have to match unless the static size is DYNAMIC!");
		}

		inline T & operator [] (int i){
			SM_ASSERT_GE_LT_DBG(std::runtime_error, i, 0, ISize, "index out of bounds");
			return p[i];
		}

		inline const T & operator [] (int i) const {
			SM_ASSERT_GE_LT_DBG(std::runtime_error, i, 0, ISize, "index out of bounds");
			return p[i];
		}
	};

	template <typename T> struct DynOrStaticSizedArray<T, Eigen::Dynamic> {
		T * p;
		DynOrStaticSizedArray(int size) : p(new T[size]){
			if(size < 0) throw new std::runtime_error("dynamic size has to be non negative");
		}
		~ DynOrStaticSizedArray(){
			delete[] p;
		}

		inline T & operator [] (int i){
			SM_ASSERT_GE_DBG(std::runtime_error, i, 0, "index out of bounds");
			return p[i];
		}

		inline const T & operator [] (int i) const {
			SM_ASSERT_GE_DBG(std::runtime_error, i, 0, "index out of bounds");
			return p[i];
		}

	};
}


#endif /* DYNAMICORTEMPLATEINT_HPP_ */
