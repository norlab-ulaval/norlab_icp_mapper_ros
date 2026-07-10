#ifndef IDESKEWER_H
#define IDESKEWER_H

#include <pointmatcher/PointMatcher.h>
#include <string>

class IDeskewer
{
  protected:
    typedef PointMatcher<float> PM;

  public:
    virtual ~IDeskewer() = default;

    virtual bool deskewCloud(PM::DataPoints &cloud, const std::string &sensorFrame) = 0;

    // Fed the rigid motion observed between the two most recent registered scans (and the time
    // elapsed between them), so that deskewing strategies which extrapolate motion (rather than
    // looking up TF) can keep a rolling estimate. No-op by default.
    virtual void updateMotion(const PM::TransformationParameters &relativeTransform, double deltaTimeSeconds)
    {
    }
};

#endif
