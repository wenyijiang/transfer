#pragma once

#include <json/json.h>
#include "sim/Controller.h"
#include "sim/Joint.h"

#include "sim/TerrainRLCharController.h"
#include "learning/MACETrainer.h"
#include "BaseControllerMACE.h"


class cPDController : public cController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eParam
	{
		eParamJointID,
		eParamKp,
		eParamKd,
		eParamTorqueLim,
		eParamTargetTheta,
		eParamTargetVel,
		eParamUseWorldCoord,
		eParamMax
	};
	typedef Eigen::Matrix<double, eParamMax, 1> tParams;

	static bool LoadParams(const std::string& file, Eigen::MatrixXd& out_buffer);
	static bool ParsePDParams(const Json::Value& root, tParams& out_params);

	cPDController();
	virtual ~cPDController();

	virtual void Init(cSimCharacter* character, const tParams& params);
	virtual void Clear();
	virtual void Update(double time_step);

	virtual cJoint& GetJoint();
	virtual const cJoint& GetJoint() const;

	virtual double GetKp() const;
	virtual void SetKp(double kp);
	virtual void SetKd(double kd);
	virtual double GetKd() const;
	virtual double GetTorqueLimit() const;
	virtual void SetTargetTheta(double theta);
	virtual void SetTargetVel(double vel);
	virtual void SetUseWorldCoord(bool use);
	virtual bool UseWorldCoord() const;

	virtual double CalcTheta() const;
	virtual double CalcVel() const;
	virtual double CalcThetaErr() const;
	virtual double CalcVelErr() const;
	virtual double CalcTargetTheta(double torque) const;
	virtual double CalcTargetVel(double torque) const;

	virtual double GetTargetTheta() const;
	virtual double GetTargetVel() const;

	virtual bool IsActive() const;

	virtual tVector CalcTorque() const;
	virtual tVector CalcTorque(const cJoint& joint) const;

protected:
	tParams mParams;
	double ts;
	
	virtual int GetJointID() const;
	
};
