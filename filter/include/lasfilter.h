#ifndef LASFILTER_H
#define LASFILTER_H

#include"lasreader.hpp"
#include"laswriter.hpp"

#include <cstdio>
#include <iostream>
#include <string>
#include"lasfilter_utils.h"

class LasFilter
{
public:
	LasFilter() = default;
	~LasFilter() = default;
	bool run(std::string szLasIn, 
		std::string szLasOut, 
		double lfLdrPtInterv,
		bool GridFilterFlag);

protected:
	bool deltGross(std::string _szLasIn, 
		std::string _szLasOut, 
		double _lfLdrPtInterv);
	bool gridFilter(std::string _szLasIn, 
		std::string _szLasOut,
		double lfDet, 
		double hight_thresh);
};
#endif
