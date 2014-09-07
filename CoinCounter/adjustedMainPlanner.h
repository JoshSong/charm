	std::vector<std::vector<std::vector<PointNode> > > genDigits(float scale);
	std::vector<PointNode> genSeg(std::vector<PointNode> points, int n);
	void writeValue(std::vector<std::vector<int> > digits, float omega);
	std::vector<PointNode> get_rotated(std::vector<PointNode> statPath,
		float omega, int dt, LONGLONG t0, int offsetX, int offsetY);