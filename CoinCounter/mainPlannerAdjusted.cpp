vector<vector<vector<PointNode> > > MainPlanner::genDigits(float scale){
	//vector<vector<PointNode> > res;

	vector<vector<vector< PointNode> > > keyPoints;
	vector<vector<PointNode> > stroke;
	vector<PointNode> temp;
	//PointNode p;
	//vector<vector<float> > strokeSize = 
	
	for (int i = 0; i < 10; ++i) {
 
 
		vector<vector<float> > xs;
		vector<vector<float> > ys;

//vector<vector<float*> > charX;
//vector<vector<float*> > charY;

		switch (i) {
			case 0:
				xs.push_back(zeroXs);
				ys.push_back(zeroYs);
				break;
			case 1:
				xs.push_back(oneXs);
				ys.push_back(oneYs);
				break;
			case 2:
				xs.push_back(twoXs);
				ys.push_back(twoYs);
				break;
			case 3:
				xs.push_back(threeXs);
				ys.push_back(threeYs);
				break;
			case 4:
				xs.push_back(fourXs);
				ys.push_back(fourYs);
				break;
			case 5:
				xs.push_back(fiveXs);
				ys.push_back(fiveYs);
				break;
			case 6:
				xs.push_back(sixXs);
				ys.push_back(sixYs);
				break;
			case 7:
				xs.push_back(sevenXs);
				ys.push_back(sevenYs);
				break;
			case 8:
				xs.push_back(eightXs);
				ys.push_back(eightYs);
				break;
			case 9:
				xs.push_back(nineXs);
				ys.push_back(nineYs);
				break;
			case 10:
				xs.push_back(mxs);
				ys.push_back(mys);
				break;
			case 11:
				xs.push_back(e1xs);
				ys.push_back(e1ys);
				xs.push_back(e2xs);
				ys.push_back(e2ys);
				break;
			case 12:
				xs.push_back(t1xs);
				ys.push_back(t1ys);
				xs.push_back(t2xs);
				ys.push_back(t2ys);
				break;
			case 13:
				xs.push_back(rxs);
				ys.push_back(rys);
				break;
			case 14:
				xs.push_back(dol1xs);
				ys.push_back(dol1ys);
				xs.push_back(dol2xs);
				ys.push_back(dol2ys);
				xs.push_back(dol3xs);
				ys.push_back(dol3ys);
				break;
			case 15:
				xs.push_back(dotxs);
				ys.push_back(dotys);
				break;
		}
		for (int i = 0; i < xs.size(); ++i) {

			PointNode p;
			//vector<PointNode> temp;
			for (int j = 0; j < xs[i].size(); ++j) {
		
				p.point.x = xs[i][j];
				p.point.y = ys[i][j];
				temp.push_back(p);
			}
			stroke.push_back(genSeg(temp, 200));
			temp.clear();
		}
 
    keyPoints.push_back(stroke);
    stroke.clear();
	}
	return keyPoints;

}


vector<PointNode> MainPlanner:: genSeg(vector<PointNode> points, int n) {
	vector<PointNode> res;

	for (int i = 0; i < points.size() - 1; ++i) {
		PointNode current = points[i];
		PointNode start = points[i];
		float dx = (points[i + 1].point.x - current.point.x) / n;
		float dy = (points[i + 1].point.y - current.point.y) / n;

		for (int j = 0; j < n; ++j) {
			current.point.x = j * dx + start.point.x;
			current.point.y = j * dy + start.point.y;
			res.push_back(current);
		}


		/*
		while (abs(current.point.x - goal.point.x) > 0.1 || abs(current.point.y < goal.point.y) > 0.1) {
			current.point.x += dx;
			current.point.y += dy;
			res.push_back(current);
		} */
	}
	res.push_back(points[points.size() - 1]);

	return res;
}

vector<PointNode> MainPlanner::get_rotated(vector<PointNode> statPath,
        float omega, int dt, LONGLONG t0, int offsetX, int offsetY) {
    vector<PointNode> res;
    for (int i = 0; i < statPath.size(); ++i) {
        PointNode temp;
        temp.point.x = (statPath[i].point.x + offsetX) * cos(omega * (i * dt + t0)/1000.f) -
                (statPath[i].point.y + offsetY) * sin(omega * (i * dt + t0)/1000.f);
        temp.point.y = (statPath[i].point.x + offsetX) * sin(omega * (i * dt + t0)/1000.f) +
                (statPath[i].point.y + offsetY) * cos(omega * (i * dt + t0)/1000.f);
        temp.time = t0 + i * dt;

        res.push_back(temp);
    }

    return res;
}



void MainPlanner:: writeValue(vector<vector<int> > digitsVector, float omega){

	float scale = 1.5;
	float x0 = -80;

	vector<vector<vector<PointNode> > > digits = genDigits(scale);
	vector<vector<vector<PointNode> > > digit_traj;
	vector<PointNode> current_traj;
	int dt = 4;
	//vector<PointNode> zero_digit = get_rotated(digits[0], 0, 1, millisecondsNow());
	for (int i = 0; i < digitsVector.size(); i++){
		for (int j = 0; j < digitsVector[i].size(); ++j) {
			digit_traj.push_back(digits[digitsVector[i][j]]);
		}
	}

	LONGLONG baseT = millisecondsNow();
	// 1.256
	for (int k = 0; k < digitsVector.size(); ++k) {
		for (int i = 0; i < digit_traj.size(); i++){
			for (int j = 0; j < digit_traj[i].size(); ++j) {
				float offsetX = (i * (2 + 0.3)) * scale + x0;
				baseT = millisecondsNow();
				current_traj = get_rotated(digit_traj[i][j], 0.566, dt, baseT, offsetX, k * 15 * scale - 30);
				vec2 first_point(current_traj[0].point.x, current_traj[0].point.y);
				arm.moveToPoint(first_point);
				Sleep(MOTOR_DELAY);
				arm.lowerArm();
				Sleep(DROP_TIME);
				arm.followTrajectory(current_traj);
				Sleep(MOTOR_DELAY);
				arm.raiseArm();
				Sleep(DROP_TIME);
		
		
			}
		}//baseT += current_traj.size() * dt + 2 * DROP_TIME + 2 * MOTOR_DELAY;
	}

}