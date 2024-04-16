#include <vector>
#include <map>

namespace RoadRunner
{
    class MultiSegment
    {
    public:
        MultiSegment(double IgnoreGap = 0);

        void Insert(double a, double b);

        std::vector<std::pair<double, double>> Merge();
    private:
        const double IgnoreGap;

        std::map<double, double> beginToDuration;
    };

    bool SegmentsIntersect(double a1, double a2, double b1, double b2);
}
