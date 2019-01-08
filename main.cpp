#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <cmath>

using namespace std;
using namespace cv;

typedef long long ll;

// Получение текущего времени в ms
ll getCTimeMillisecond()
{
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);
	return (long long)(time.tv_sec) * 1000 + time.tv_nsec / 1000000;
}

struct CollisionSLine
{
	Point2d p1, p2; //p1 - begin(old), p2 - end(new)
	double k;

	CollisionSLine() : k(0) {};

	CollisionSLine(double x1, double y1, double x2, double y2, double k) : p1(x1, y1), p2(x2, y2), k(k) {};

	// Смотрит пересекаются ли отрезки
	bool tryCross(CollisionSLine l)
	{
		double a_x = p1.x - p2.x;
		double a_y = p1.y - p2.y;
		double vp_1 = a_x * (p1.y - l.p1.y) - (p1.x - l.p1.x) * a_y;
		double vp_2 = a_x * (p1.y - l.p2.y) - (p1.x - l.p2.x) * a_y;
		if ((vp_1 <= 0 && vp_2 < 0) || (vp_1 >= 0 && vp_2 > 0)) return false;

		a_x = l.p1.x - l.p2.x;
		a_y = l.p1.y - l.p2.y;
		vp_1 = a_x * (l.p1.y - p1.y) - (l.p1.x - p1.x) * a_y;
		vp_2 = a_x * (l.p1.y - p2.y) - (l.p1.x - p2.x) * a_y;
		if ((vp_1 <= 0 && vp_2 < 0) || (vp_1 >= 0 && vp_2 > 0)) return false;

		return true;
	}

	// Ищет точку перечечения (перпенликуляра от конечной точки траектории к прямой от которой нужно отразить)
	Point2d getCrossPoint(CollisionSLine lin)
	{
		double d, dx, dy, x = 0, y = 0;

		double A = p1.y - p2.y;
		double B = p2.x - p1.x;
		double C = -A * p1.x - B * p1.y;

		double Alin = lin.p1.y - lin.p2.y;
		double Blin = lin.p2.x - lin.p1.x;
		double Clin = -Alin * lin.p1.x - Blin * lin.p1.y;

		d = A * Blin - Alin * B;
		dx = -C * Blin + Clin * B;
		dy = -A * Clin + Alin * C;


		if (d == 0)
		{
			if (dx == 0 && dy == 0) return lin.p1; // Линии совпадают
//			if (dx != 0 && dy != 0);
		}
		else
		{
			x = dx / d;
			y = dy / d;
		}

		return Point2d(x, y);
	}

	// Ищет расстояние от точки до прямой
	Point2d getNeirousPoint(Point2d poi)
	{
		double A = p1.y - p2.y;
		double B = p2.x - p1.x;
		double C = -A * p1.x - B * p1.y;

		double nc = B * poi.x - A * poi.y;
		double d = A * A + B * B;
		double dx = -C * A + nc * B;
		double dy = -A * nc - B * C;

		return Point2d(dx / d, dy / d);
	}
};


struct SegmentLine
{
	CollisionSLine data;
	Scalar color;
	int thickness;

	SegmentLine() : thickness(0) {};

	SegmentLine(double x1, double y1, double x2, double y2, double k, Scalar color, int thickness) : data(x1, y1, x2, y2, k), color(color), thickness(thickness) {};

	void drawLine(Mat img)
	{
		line(img, Point(data.p1.x * 80, 800 - data.p1.y * 80), Point(data.p2.x * 80, 800 - data.p2.y * 80), color, thickness, CV_AA);
	}

};

struct SCircle
{
	vector<CollisionSLine> vecData;
	Point2d center;
	Scalar color;
	int thickness;
	double radius;

	SCircle() : thickness(0), radius(0) {};

	SCircle(double px, double py, double r, double k, Scalar color, int thickness, int quality)
	{
		center.x = px;
		center.y = py;
		this->color = color;
		radius = r;
		this->thickness = thickness;

		double al = 2 * M_PI / quality;

		// Создание окружности для расчета физики
		vecData.resize(quality);
		vecData[0].p1.x = px + r;
		vecData[0].p1.y = py;

		for (unsigned int i = 1; i < vecData.size(); i++)
		{
			double cal = al * i;
			vecData[i - 1].p2.x = cos(cal) * r + px;
			vecData[i - 1].p2.y = sin(cal) * r + py;
			vecData[i].p1.x = vecData[i - 1].p2.x;
			vecData[i].p1.y = vecData[i - 1].p2.y;
			vecData[i].k = k;
		}
		vecData[vecData.size() - 1].p2.x = vecData[0].p1.x;
		vecData[vecData.size() - 1].p2.y = vecData[0].p1.y;

	};

	void drawCircle(Mat img)
	{
		circle(img, Point(center.x * 80, 800 - center.y * 80), radius * 80, color, thickness, CV_AA);
	}

};


struct Ball
{
	CollisionSLine data; // p2 - now pos
	Point2d speed;
	Scalar color;
	bool isAct = true; // Существование мячика
	int size;

	Ball() : size(0) {};

	Ball(double x, double y, double sx, double sy, double k, Scalar color, int size): data(x, y, x, y, k), speed(sx, sy), color(color), size(size) {};

	// Считает следующий шаг
	void nextStep(int dTime, vector<CollisionSLine>& vecLine)
	{
		if(!isAct) return;

		// Разбиванем расчет движения мячика на промежутки длинной 1ms
		for (int j = 0; j < dTime; j++){
			data.p1 = data.p2;

			double lTime = 1 / 1000.0;

			// Симуляция силы притяжения
			speed.y += -9.8 * lTime;
			data.p2.x += speed.x * lTime;
			data.p2.y += speed.y * lTime;

			// Если мячик вышел за край экрана, то он удаляется
			if (data.p2.y < -5)
				isAct = false;

			unsigned int miId = -1;

			while(true)
			{
				double mi = 999999;
				bool isCross = false;
				Point2d crossMi;

				// Поиск ближайшей линии от которой произойдет отскок
				for (unsigned int i = 0; i < vecLine.size(); i++)
				{
					// Мячик не может 2 раза отскочить от одной линии
					if (i == miId) continue;

					// Если не было контакта с линией, то переходим к следующей
					if (!data.tryCross(vecLine[i])) continue;
					isCross = true;

					// Вычисление точки пересечения траектории мячика и линии
					Point2d crossPoint = data.getCrossPoint(vecLine[i]);
					double dx = data.p1.x - crossPoint.x;
					double dy = data.p1.y - crossPoint.y;
					double d = dx * dx + dy * dy;
					if (d < mi)
					{
						mi = d;
						miId = i;
						crossMi = crossPoint;
					}
				}

				if (isCross)
				{
					// Находим координаты мячика после отскока
					Point2d res = vecLine[miId].getNeirousPoint(data.p2);
					data.p2.x = 2 * res.x - data.p2.x;
					data.p2.y = 2 * res.y - data.p2.y;

					// Считаем sin и cos угла наклона линии
					double sinA, cosA, dA, xA, yA;
					xA = vecLine[miId].p2.x - vecLine[miId].p1.x;
					yA = vecLine[miId].p2.y - vecLine[miId].p1.y;
					dA = sqrt(xA * xA + yA * yA);
					sinA = yA / dA;
					cosA = xA / dA;

					// Перевод вектора движения шарика в систему координат связанную с линией
					double nx, ny;
					nx = speed.x * cosA + sinA * speed.y;
					ny = speed.y * cosA - sinA * speed.x;

					ny = -ny * vecLine[miId].k * data.k; // Скорость после отталкивания
					nx *= 0.99999; // Симуляция силы трения

					// Перевод в обычную систему координат
					speed.x = nx * cosA - sinA * ny;
					speed.y = ny * cosA + sinA * nx;

				}
				else break;
			}
		}

	}

	void drawBall(Mat img)
	{
		if (!isAct) return;
		circle(img, Point(data.p2.x * 80, 800 - data.p2.y * 80), size, color, -1, CV_AA);
	}

	// Генерирует мячик в случайном месте
	void genRand()
	{
		data.p2.x = rand() % 100 / 10.0;
		data.p2.y = rand() % 100 / 10.0;
		data.p1.x = data.p2.x;
		data.p1.y = data.p2.y;

		genRandSpeed();
	}

	void genRandSpeed()
	{
		speed.x = (rand() % 60 - 30) / 10.0;
		speed.y = (rand() % 60 - 30) / 10.0;
	}

};

vector<Ball> vecBall;

void mouseClick(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		Ball ball(x / 80.0, (800 - y) / 80.0, 0, 0, 0.98, Scalar(255, 158, 60), 6);
		ball.genRandSpeed();
		vecBall.push_back(ball);
	}
}

void addToMap(vector<CollisionSLine>& out, CollisionSLine in)
{
	out.push_back(in);
}

void addToMap(vector<CollisionSLine>& out, vector<CollisionSLine>& in)
{
	int si = out.size();

	out.resize(si + in.size());

	for (unsigned int i = si; i < out.size(); i++)
		out[i] = in[i - si];
}

int main()
{
	srand(getCTimeMillisecond());

	namedWindow("window");
	setMouseCallback("window", mouseClick);

	Mat mainFrame(800, 800, CV_8UC3, Scalar(200, 200, 200));

	vector<CollisionSLine> vecLine;

	SegmentLine lines[25];

	Scalar colorBike(20, 180, 20);
	lines[0] = SegmentLine(2, 2, 2.75, 5, 0.5, colorBike, 2);
	lines[1] = SegmentLine(2.75, 5, 3, 6, 0.5, colorBike, 2);
	lines[2] = SegmentLine(2.5, 6, 3, 6, 0.5, colorBike, 2);
	lines[3] = SegmentLine(2.75, 5, 7, 5, 0.5, colorBike, 2);
	lines[4] = SegmentLine(7, 5, 7.33, 5.5, 0.5, colorBike, 2);
	lines[5] = SegmentLine(7, 5.5, 7.5, 5.5, 0.5, colorBike, 2);
	lines[6] = SegmentLine(7, 5, 8, 2, 0.5, colorBike, 2);
	lines[7] = SegmentLine(7, 5, 5.27, 2.41, 0.5, colorBike, 2);
	lines[8] = SegmentLine(8, 2, 5.5, 2, 0.5, colorBike, 2);
	lines[9] = SegmentLine(4.72, 2.38, 2.75, 5, 0.5, colorBike, 2);
	lines[10] = SegmentLine(5, 1.5, 5.25, 1, 0.5, colorBike, 2);
	lines[11] = SegmentLine(5, 1, 5.5, 1, 0.5, colorBike, 2);

	Scalar colorCrutch(180, 20, 20);
	lines[12] = SegmentLine(1.57, 9.72, 1, 8, 0.5, colorCrutch, 2);
	lines[13] = SegmentLine(1.57, 9.72, 2.24, 9.5, 0.5, colorCrutch, 2);
	lines[14] = SegmentLine(2.24, 9.5, 2.17, 9.28, 0.5, colorCrutch, 2);
	lines[15] = SegmentLine(2.17, 9.28, 1.73, 8, 0.5, colorCrutch, 2);
	lines[16] = SegmentLine(1.73, 8, 1.66, 7.77, 0.5, colorCrutch, 2);
	lines[17] = SegmentLine(1, 8, 1.66, 7.77, 0.5, colorCrutch, 2);
	lines[18] = SegmentLine(2.17, 9.28, 4.71, 8.44, 0.5, colorCrutch, 2);
	lines[19] = SegmentLine(4.71, 8.44, 4.28, 7.14, 0.5, colorCrutch, 2);
	lines[20] = SegmentLine(4.28, 7.14, 1.73, 8, 0.5, colorCrutch, 2);
	lines[21] = SegmentLine(4.71, 8.44, 5.81, 7.35, 0.5, colorCrutch, 2);
	lines[22] = SegmentLine(4.28, 7.14, 5.81, 7.35, 0.5, colorCrutch, 2);
	lines[23] = SegmentLine(8.15, 6.57, 5.81, 7.35, 0.5, colorCrutch, 2);
	lines[24] = SegmentLine(8.56, 6.8, 8.34, 6.14, 0.5, colorCrutch, 2);

	SCircle circle1(2, 2, 1.5, 0.5, Scalar(20, 20, 200), 2, 150);
	SCircle circle2(8, 2, 1.5, 0.5, Scalar(20, 20, 200), 2, 150);
	SCircle circle3(5, 2, 0.5, 0.5, colorBike, 2, 150);

	circle1.drawCircle(mainFrame);
	circle2.drawCircle(mainFrame);
	circle3.drawCircle(mainFrame);

	for (SegmentLine& l : lines)
	{
		l.drawLine(mainFrame);
		addToMap(vecLine, l.data);
	}

	addToMap(vecLine, circle1.vecData);
	addToMap(vecLine, circle2.vecData);
	addToMap(vecLine, circle3.vecData);


	ll lastTime = getCTimeMillisecond();

	while (true)
	{
		ll currentTime = getCTimeMillisecond();

		int dTime = currentTime - lastTime;
		lastTime = currentTime;

		Mat frame = mainFrame.clone();

		for (Ball& ball : vecBall)
		{
			ball.nextStep(dTime, vecLine);
			ball.drawBall(frame);
		}

		imshow("window", frame);
		waitKey(33);
	}

	return 0;
}
