#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
#define MIN_VALUE 1e-8

double multiply(Point& p1, Point& p2, Point& p0) {
	double result = (p1.x - p0.x) * (p2.y - p0.y)*1.0 - (p1.y - p0.y) * (p2.x - p0.x)*1.0;
	//cout<<result<<endl;
	return result;
}

bool inConvexPolygon(std::vector<Point> Polygon, Point target) {
	int len = Polygon.size();
	if (multiply(target, Polygon[1], Polygon[0]) >-MIN_VALUE || multiply(target, Polygon[len - 1], Polygon[0]) <MIN_VALUE) {
		return false;
	}
	int s = 1, e = len - 1;
	int line = -1;
	while (s <= e) {
		int m = (e + s) >> 1;
		//cout<<e<<","<<s<<","<<m<<endl;
		if (multiply(target, Polygon[m], Polygon[0]) > 0) { // target在m顺时针方向
			line = m; // line
			e = m - 1;
		}
		else { // target
			s = m + 1;
		}
	}
	//cout<<line<<endl;
	double sum = multiply(Polygon[line], target, Polygon[line - 1]);
	bool result = sum>MIN_VALUE;
	return result;
}

void getLocations(Size sz, std::vector<std::vector<Point> > &locations, std::vector<Point>pts,
	double scale, Size winStride, Size padding, Size winSize,int levels)
{
	double nowscale = 1.0;
	int currentlevel = 1;
	while (currentlevel<=levels)
	{
		std::vector<Point> scalePoints;
		std::vector<std::vector<int> > region;
		std::vector<std::vector<bool> > windows;
		std::vector<Point> location;
		for (int i = 0; i < pts.size(); i++) {
			Point ps = Point((int)((double)pts[i].x / nowscale), (int)((double)pts[i].y / nowscale));
			scalePoints.push_back(ps);
		}
		int width = int((double)sz.width/nowscale)/ winStride.width;
		int height = int((double)sz.width / nowscale) / winStride.height;

		//BREAK;
		if (cvRound(sz.width / scale) < winSize.width ||
			cvRound(sz.height / scale) < winSize.height ||
			scale <= 1)
			break;

		//init region
		for (int i = 0; i < width; i++) {
			std::vector<int> list;
			for (int j = 0; j < height; j++) {
				list.push_back(0);
			}
			region.push_back(list);
		}

		//get regions;
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				Point ps;
				ps = Point(i * winStride.width, j * winStride.height);
				bool isContain = inConvexPolygon(scalePoints, ps);
				if (isContain)
				{
					region[i][j] = 2;
					if (i > 0)
						region[i - 1][j] = 1;
					if (i < width - 1)
						region[i + 1][j] = 1;
					if (j > 0)
						region[i][j - 1] = 1;
					if (j < height - 1)
						region[i][j + 1] = 1;
				}
			}
		}

		//init windows.
		int paddingx = padding.width / winStride.width;
		int paddingy = padding.height / winStride.height;
		//BLOCK。
		int blockx = winSize.width / winStride.width;
		int blocky = winSize.height / winStride.height;
		for (int nx = 0; nx <= width - blockx + 2 * paddingx; nx++) {
			std::vector<bool> list;
			for (int ny = 0; ny <= height - blocky + 2 * paddingy; ny++) {
				list.push_back(true);
			}
			windows.push_back(list);
		}

		for (int nx = 0; nx <= width - blockx + 2 * paddingx; nx++) {
			for (int ny = 0; ny <= height - blocky + 2 * paddingy; ny++) {
				int currentx = nx - paddingx;
				int currenty = ny - paddingy;
				if ((currentx >= 0) && (currenty >= 0) && (!region[currentx][currenty]))
				{
					windows[nx][ny] = false;
				}
				else if (windows[nx][ny])
				{
					//speed up,4 states
					int state = 0;
					if ((nx > 0) && (windows[nx - 1][ny]))
						state |= 1;
					if ((ny > 0) && (windows[nx][ny - 1]))
						state |= 2;
					//only one block;
					//cout<<nx<<","<<ny<<","<<state<<endl;
					if (state == 3)
					{
						if ((currenty + blocky - 1 < height) && (currentx + blockx - 1 < width)
							&& (!region[currentx + blockx - 1][currenty + blocky - 1]))
							windows[nx][ny] = false;
					}
					else if (state == 2)
					{
						for (int searchindex = blockx - 1; searchindex >= 0; searchindex--) {
							if ((currenty + blocky - 1 >= height) || (currentx + searchindex < 0))
								break;
							if (currentx + searchindex >= width)
								continue;
							if (!region[currentx + searchindex][currenty + blocky - 1])
							{
								for (int maskx = 0; maskx <= searchindex; maskx++) {
									if (currentx + maskx <= width - blockx + paddingx)
										windows[nx + maskx][ny] = false;
								}
								break;
							}
						}
					}
					else if (state == 1)
					{
						for (int searchindey = blocky - 1; searchindey >= 0; searchindey--) {
							if ((currentx + blockx - 1 >= width) || (currenty + searchindey < 0))
								break;
							if (currenty + searchindey >= height)
								continue;
							if (!region[currentx + blockx - 1][currenty + searchindey])
							{
								for (int masky = 0; masky <= searchindey; masky++) {
									if (currenty + masky <= height - blocky + paddingy)
										windows[nx][ny + masky] = false;
								}
								break;
							}
						}
					}
					else {
						for (int searchindex = blockx * blocky - 1; searchindex >= 0; searchindex--) {
							int indexx = searchindex / blocky + currentx;
							int indexy = searchindex % blocky + currenty;
							//set block
							if ((indexx < 0) || (indexy < 0) || (indexx >= width) || (indexy >= height))
								continue;
							if (!region[indexx][indexy])
							{
								for (int maskx = 0; maskx < blockx; maskx++) {
									if ((indexx - maskx < currentx) || (indexx - maskx > width - blockx + paddingx))
										continue;
									for (int masky = 0; masky < blocky; masky++) {
										if ((indexy - masky < currenty) || (indexy - masky > height - blocky + paddingy))
											continue;
										windows[indexx - maskx + paddingx][indexy - masky + paddingy] = false;
									}
								}
								break;

							}
						}
					}
				}
			}
		}

		//OPENCV CACHE的顺序是Y先不变，所以存储顺序要适应。。。。否则CACHE有问题。
		for (int ny = 0; ny <= height - blocky + 2 * paddingy; ny++) {
			for (int nx = 0; nx <= width - blockx + 2 * paddingx; nx++) {
				int currentx = nx - paddingx;
				int currenty = ny - paddingy;
				if (windows[nx][ny])
					location.push_back(Point(currentx*winStride.width,
						currenty*winStride.height));
			}
		}
		locations.push_back(location);
		nowscale *= scale;
		currentlevel++;
	}
}