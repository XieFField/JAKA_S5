#ifndef MASSAGE_MOTION__COMPLIANCE_CONTROLLER_HPP_
#define MASSAGE_MOTION__COMPLIANCE_CONTROLLER_HPP_

#include "massage_motion/compliance_types.hpp"

namespace massage_motion
{

// 柔顺控制器与轨迹规划器是并列能力，不存在继承关系。
// 仿真控制器和 JAKA 真机适配器都必须遵守此接口。
class IComplianceController
{
public:
    virtual ~IComplianceController() = default;

    virtual ComplianceResult start(
        const ComplianceRequest & request) = 0;

    virtual ComplianceResult stop() = 0;

    virtual bool update_reference(
        const ComplianceReference & reference) = 0;

    virtual ComplianceFeedback feedback() const = 0;

    virtual bool reset() = 0;

    virtual ComplianceStatus status() const = 0;
};

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__COMPLIANCE_CONTROLLER_HPP_
