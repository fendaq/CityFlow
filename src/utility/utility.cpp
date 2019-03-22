#include "utility/utility.h"
#include <cmath>
#include <algorithm>

namespace CitySimulator {
	
	Point operator*(const Point &A, double k) {
		return {A.x * k, A.y * k};
	}

	Point operator-(const Point &A, const Point &B) {
		return {A.x - B.x, A.y - B.y};
	}

	Point operator+(const Point &A, const Point &B) {
		return {A.x + B.x, A.y + B.y};
	}
    Point operator-(const Point &A) {
	    return {-A.x, -A.y};
	}

    Point calcIntersectPoint(Point A, Point B, Point C, Point D) {
	    Point P = A;
	    Point Q = C;
	    Vector u = B - A;
	    Vector v = D - C;
	    return P + u * (crossMultiply(Q - P, v) / crossMultiply(u, v));
    }


    double crossMultiply(const Point &A, const Point &B) {
	    return A.x * B.y - A.y * B.x;
    }

    double dotMultiply(const Point &A, const Point &B) {
	    return A.x * B.x + A.y * B.y;
    }

	double calcAng(Point A, Point B) {
		double ang = A.ang() - B.ang();
		double pi = acos(-1);
		while(ang >= pi / 2)
			ang -= pi / 2;
		while(ang < 0)
			ang += pi / 2;
		return std::min(ang, pi - ang);
	}

    bool onSegment(Point A, Point B, Point P) {
	    double v1 = crossMultiply(B-A, P-A);
	    double v2 = dotMultiply(P-A, P-B);
	    return Point::sign(v1) == 0 && Point::sign(v2) <= 0;
    }

    Point Point::unit() {
	    double l = len();
	    return {x / l, y / l};
    }

    Point Point::normal() {
	    return {-y, x};
    }

    Point::Point(double x, double y):x(x),y(y) { }

    double Point::len() {
	    return sqrt(x * x + y * y);
    }

	double Point::ang() {
		return atan2(y, x);
	}

    int Point::sign(double x) {
        return (x + Point::eps > 0) - (x < Point::eps);
    }

	std::vector<int> generateRandomIndices(size_t n, std::mt19937 &rnd) {
		std::vector<int> randoms;
		randoms.reserve(n);
		for (size_t i = 0; i < n; ++i) {
			randoms.emplace_back(i);
		}
		std::shuffle(randoms.begin(), randoms.end(), rnd);
		return randoms;
	}
}