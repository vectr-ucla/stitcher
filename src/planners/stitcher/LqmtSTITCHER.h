#ifndef LQMT_STITCHER_H
#define LQMT_STITCHER_H

#include <LQMTPrimitive.h>
#include <STITCHER.h>

namespace Motion_Primitives {
    class LqmtSTITCHER;
}

class Motion_Primitives::LqmtSTITCHER
    : public Motion_Primitives::STITCHER<Motion_Primitives::LQMTPrimitive> {
public:
    
    LqmtSTITCHER();
    virtual ~ LqmtSTITCHER();
    bool generatePrimitive(Motion_Primitives::LQMTPrimitive &primitive, 
                           const Eigen::MatrixXd &current_state, 
                           const Eigen::MatrixXd &des_state) override;
};

#endif