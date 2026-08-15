#ifndef MASSAGE_MOTION__COMPLIANCE_VALIDATION_HPP_
#define MASSAGE_MOTION__COMPLIANCE_VALIDATION_HPP_

#include "massage_motion/compliance_types.hpp"

namespace massage_motion
{

ComplianceValidationResult validate_compliance_request(
    const ComplianceRequest & request);

}  // namespace massage_motion

#endif  // MASSAGE_MOTION__COMPLIANCE_VALIDATION_HPP_
