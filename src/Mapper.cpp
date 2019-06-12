#include "Mapper.h"

Mapper::Mapper()
{
}

void Mapper::updateMap(const PM::DataPoints& cloud)
{
	// Do interesting stuff here
}

const PM::DataPoints& Mapper::getMap()
{
	return map;
}
