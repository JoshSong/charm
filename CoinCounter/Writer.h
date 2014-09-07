
#pragma once

#include "rowmans.h"
#include "PointNode.h"
#include "Point.h"
#include "Globals.h"


class Writer {

public:

	void parseGlyph(float speed);

	std::vector< std::vector< PointNode > > getCharTraj(Point start, LONGLONG char_time, LONGLONG start_time, char letter);

	std::vector< std::vector< std::vector < PointNode > > > dictionary;



};