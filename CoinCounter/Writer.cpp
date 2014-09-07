

#include "stdafx.h"
#include "Writer.h"

using namespace std;

void Writer::parseGlyph(float speed) {

	dictionary.reserve(rowmans_count);

	for (int i = 0; i < rowmans_count; i++) {

		dictionary.push_back(vector< vector<PointNode> >());

		LONGLONG time = 0;

		dictionary.back().push_back(vector<PointNode>());
		Point P0(rowmans[i][0], rowmans[i][1]);
		Point P1(rowmans[i][2], rowmans[i][3]);
		dictionary.back().back().push_back(PointNode(P0, time));
		time += (P0.distance(P1)/speed);
		dictionary.back().back().push_back(PointNode(P1, time));

		Point prevPoint = P1;

		for (int j = 4; j < rowmans_size[i]; j+=4) {
			Point P0(rowmans[i][j], rowmans[i][j+1]);
			Point P1(rowmans[i][j+2], rowmans[i][j+3]);

			if (!P0.equals(prevPoint)) {
				dictionary.back().push_back(vector<PointNode>());
				time += 2 * DROP_TIME + prevPoint.distance(P0)/time;
				dictionary.back().back().push_back(PointNode(P0, time));
			}

			time += (P0.distance(P1)/speed);
			dictionary.back().back().push_back(PointNode(P1, time));
		}
	}
}

vector< vector< PointNode > > Writer::getCharTraj(Point start, LONGLONG char_time, LONGLONG start_time, char letter) {

	int index = letter - 32;

	if (index < 0 || index >= 96) {
		printf("Character out of range \n");
		return vector<vector<PointNode> >();
	}

	vector< vector< PointNode > > traj;

	for (int i = 0; i < dictionary[index].size(); i++) {
		traj.push_back(vector<PointNode>());

		for (int j = 0; j < dictionary[index][i].size(); j++) {
			PointNode temp = dictionary[index][i][j];
			temp.point.x += start.x;
			temp.point.y += start.y;
			temp.time += char_time;

			float t = (temp.time - start_time)/1000.f;
			temp.point.x = temp.point.x * cos(ROT_SPEED * t) - temp.point.y * sin(ROT_SPEED * t);
			temp.point.y = temp.point.x * sin(ROT_SPEED * t) + temp.point.y * cos(ROT_SPEED * t);

			traj.back().push_back(temp);
		}
	}
	return traj;
}



