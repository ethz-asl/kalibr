#ifndef SM_STRING_ROUTINES_H
#define SM_STRING_ROUTINES_H

#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>
namespace sm
{
    /// replaces the 'target' with 'replacement' searching forward through
    /// the string and skipping 'skip' occurences
    inline std::string replace(std::string str, char target, char replacement, unsigned int skip = 0)
    {
        std::string::size_type pos = 0;
        while( (pos = str.find(target, pos)) != std::string::npos)
        {
            if(skip)
            {
                skip--;
                pos++;
            }
            else
                str[pos] = replacement;
        }
        return str;
    }

    /// replaces the 'target' with 'replacement' searching backward through
    /// the string and skipping 'skip' occurences
    inline std::string replace_reverse(std::string str, char target, char replacement, unsigned int skip = 0)
    {
        std::string::size_type pos = std::string::npos;
        while( (pos = str.rfind(target, pos)) != std::string::npos)
        {
            if(skip)
            {
                skip--;
                pos--;
            }
            else
                str[pos] = replacement;
        }
        return str;
    }

    inline std::string leading_pad( std::string str, unsigned int desiredLength, char padChar )
    {
        std::string result;
        if ( str.length() < desiredLength )
        {
            std::string padding( desiredLength - str.length(), padChar );
            result = padding.append(str);
        }
        else
        {
            result = str;
        }
        return result;
    }

    /*
    inline std::string tolower(std::string s) {
        for(unsigned int i = 0; i < s.size(); i++)
            s[i] = ::tolower(s[i]); // call tolower from the c std lib.
        return s;
    }
    */
    // Find all substrings of the form $(XXX) and replace them
    // with the associated environment variable if they exist.
    inline std::string substituteEnvVars(std::string const & s) {
        std::string::size_type off = 0, pidx=0, idx=0, idxe=0;
        std::ostringstream out;
        idx = s.find("$(",off);
        while(idx != std::string::npos){
            //std::cout << "Found \"$(\" at " << idx << std::endl;
            idxe = s.find(')',idx);
            if(idxe != std::string::npos) {
                //std::cout << "Found \")\" at " << idxe << std::endl;
                // We've found an environment variable.
                std::string envVar = s.substr(idx+2,idxe-idx-2);
                //std::cout << "evname: " << envVar << std::endl;
                char * envSub = getenv(envVar.c_str());

                if(envSub != NULL) {
                    // Dump everything before the envVar
                    out << s.substr(pidx,idx-pidx);
                    out << envSub;
                    pidx = idxe+1;
                }
                off = idxe+1;
                idx = s.find("$(",off);
            } else {
                // No close brackets means no env vars.
                idx = std::string::npos;
            }

        }
        out << s.substr(pidx);
        return out.str();
    }


    inline std::string ensureTrailingBackslash(std::string const & s)
    {
        if(s.size() == 0)
            return "/";
    
        if(s[s.size()-1] == '/')
            return s;
    
        return s + "/";
    }

	// a nice and short to-string function.
	template<typename T>
	std::string ts(T t)
	{
		std::ostringstream s;
		s << t;
		return s.str();
	}


	// pad an int with zeros.
    template<typename T>
    inline std::string padded(const T & val, unsigned int digits, char fillChar = '0')
	{
		std::ostringstream s;
		s.fill(fillChar);
		s.width(digits);
		s << val;
		return s.str();
	}


	inline std::string padded(double val, unsigned width, unsigned precision) {
		std::ostringstream s;
		s.fill(' ');
		s.width(width);
		s.setf(std::ios::fixed,std::ios::floatfield);   // floatfield set to fixed
		s.precision(precision);
		s << val;

		return s.str();
	}



	inline std::string scientific(double val, unsigned width, unsigned precision) {
		std::ostringstream s;
		s.fill(' ');
        /// The min width will be the precision plus
        /// (the leading -) + (1.) + (e-02) = 7 extra characters
		s.width(std::max(width,precision + 7));
		s.setf(std::ios::scientific,std::ios::floatfield);   // floatfield set to fixed
		s.precision(precision);
		s << val;

		return s.str();
	}

	inline std::string fixedFloat(double val, unsigned precision) {
		std::ostringstream s;
		s.precision(precision);
		s << std::fixed << val;
		return s.str();
	}

	template<typename T>
	std::string s_tuple(T t)
	{
		std::ostringstream s;
		s << t;
		return s.str();
	}

	template<typename T, typename U>
	std::string s_tuple(T t, U u)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "]";
		return s.str();
	}

	template<typename T, typename U, typename V>
	std::string s_tuple(T t, U u, V v)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "," << v << "]";
		return s.str();
	}

	template<typename T, typename U, typename V, typename W>
	std::string s_tuple(T t, U u, V v, W w)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "," << v << "," << w << "]";
		return s.str();
	}

	template<typename T, typename U, typename V, typename W, typename X>
	std::string s_tuple(T t, U u, V v, W w, X x)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "," << v << "," << w << "," << x <<"]";
		return s.str();
	}

	template<typename T, typename U, typename V, typename W, typename X, typename Y>
	std::string s_tuple(T t, U u, V v, W w, X x, Y y)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "," << v << "," << w << "," << x << "," << y << "]";
		return s.str();
	}

	template<typename T, typename U, typename V, typename W, typename X, typename Y, typename Z>
	std::string s_tuple(T t, U u, V v, W w, X x, Y y, Z z)
	{
		std::ostringstream s;
		s << "[" << t << "," << u << "," << v << "," << w << "," << x << "," << y << "," << z << "]";
		return s.str();
	}




	template<typename ConstIterator>
	inline void toStream(std::ostream & stream, ConstIterator begin, ConstIterator end, std::string delimiter = " ", std::string prefix = "", std::string postfix = "")
	{
		ConstIterator i = begin;
		stream << prefix;
		if(i != end)
		{
			stream << *i;
			i++;
			for( ; i != end; ++i)
				stream << delimiter << *i;
		}
		stream << postfix;
	}


	template<typename Iterator>
	std::string arrToString(Iterator start, Iterator end)
	{
        std::ostringstream s;
        toStream(s, start, end, ",", "[", "]");
        return s.str();
	}

	template<typename T>
	std::string arrToString(T * thearr, unsigned int size)
	{
          
        return arrToString((T*)thearr, (T*)(thearr + size));
	}



	template<typename ConstIterator>
	inline void paddedToStream(std::ostream & stream, int padLength, int decimalLength, ConstIterator begin, ConstIterator end, std::string delimiter = " ", std::string prefix = "", std::string postfix = "")
	{
		stream << prefix;

		ConstIterator i = begin;

		if(i != end)
		{
			stream.fill(' ');
			stream.width(padLength);
			stream.setf(std::ios::fixed,std::ios::floatfield);   // floatfield set to fixed
			stream.precision(decimalLength);

			stream << *i;
			i++;
			for( ; i != end; ++i){
				stream << delimiter;
				stream.fill(' ');
				stream.width(padLength);
				stream.setf(std::ios::fixed,std::ios::floatfield);   // floatfield set to fixed
				stream.precision(decimalLength);

				stream << *i;
			}
		}
		stream << postfix;
	}


	template<typename ConstIterator>
	inline std::string paddedToString( int padLength, int decimalLength, ConstIterator begin, ConstIterator end, std::string delimiter = " ", std::string prefix = "", std::string postfix = "")
	{
        std::ostringstream stream;
        paddedToStream(stream,padLength,decimalLength,begin,end,delimiter,prefix,postfix);
        return stream.str();
    }


} // namespace sm


#endif

