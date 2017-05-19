#include "PDController.h"
#include <iostream>
#include "sim/SimCharacter.h"
#include "util/FileUtil.h"
#include<vector>
#include"math.h"
const std::string gPDControllersKey = "PDControllers";
const std::string gPDParamKeys[cPDController::eParamMax] =
{
	"JointID",
	"Kp",
	"Kd",
	"TorqueLim",
	"TargetTheta",
	"TargetVel",
	"UseWorldCoord"
};

bool cPDController::LoadParams(const std::string& file, Eigen::MatrixXd& out_buffer)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gPDControllersKey].isNull())
		{
			const Json::Value& pd_controllers = root[gPDControllersKey];
			int num_ctrls = pd_controllers.size();
			out_buffer.resize(num_ctrls, eParamMax);

			for (int i = 0; i < num_ctrls; ++i)
			{
				tParams curr_params;
				Json::Value json_pd_ctrl = pd_controllers.get(i, 0);
				bool succ_def = ParsePDParams(json_pd_ctrl, curr_params);
				if (succ_def)
				{
					int joint_id = i;
					curr_params[eParamJointID] = i;
					out_buffer.row(i) = curr_params;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
		
	}
	else
	{
		printf("Failed to load PD controller parameters from %s\n", file.c_str());
	}

	return succ;
}

bool cPDController::ParsePDParams(const Json::Value& root, tParams& out_params)
{
	bool succ = true;

	out_params.setZero();
	for (int i = 0; i < eParamMax; ++i)
	{
		const std::string& curr_key = gPDParamKeys[i];
		if (!root[curr_key].isNull() && root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_params[i] = val;
		}
	}

	return succ;
}

cPDController::cPDController()
	: cController()
{
	Clear();
}

cPDController::~cPDController()
{
}

void cPDController::Init(cSimCharacter* character, const tParams& params)
{
	cController::Init(character);
	mParams = params;
	mValid = true;

	cJoint& joint = GetJoint();
	double torque_lim = GetTorqueLimit();
	joint.SetTorqueLimit(torque_lim);
}

void cPDController::Clear()
{
	cController::Clear();
	
	mParams[eParamJointID] = static_cast<double>(cKinTree::gInvalidJointID);
	mParams[eParamKp] = 0;
	mParams[eParamKd] = 0;
	mParams[eParamTorqueLim] = std::numeric_limits<double>::infinity();
	mParams[eParamTargetTheta] = 0;
	mParams[eParamTargetVel] = 0;
	mParams[eParamUseWorldCoord] = 0;
}

void cPDController::Update(double time_step)
{
        ts=time_step;
	if (IsActive())
	{
		cJoint& joint = GetJoint();
		tVector torque = CalcTorque();
		joint.AddTorque(torque);
	}
}

cJoint& cPDController::GetJoint()
{
	return mChar->GetJoint(GetJointID());
}

const cJoint& cPDController::GetJoint() const
{
	return mChar->GetJoint(GetJointID());
}

void cPDController::SetKp(double kp)
{
	mParams[eParamKp] = kp;
}

double cPDController::GetKp() const
{
	return mParams[eParamKp];
}

double cPDController::GetTorqueLimit() const
{
	return mParams[eParamTorqueLim];
}

void cPDController::SetKd(double kd)
{
	mParams[eParamKd] = kd;
}

double cPDController::GetKd() const
{
	return mParams[eParamKd];
}

void cPDController::SetTargetTheta(double theta)
{
	mParams[eParamTargetTheta] = theta;
}

void cPDController::SetTargetVel(double vel)
{
	mParams[eParamTargetVel] = vel;
}

void cPDController::SetUseWorldCoord(bool use)
{
	mParams[eParamUseWorldCoord] = (use) ? 1 : 0;
}

bool cPDController::UseWorldCoord() const
{
	return mParams[eParamUseWorldCoord] != 0;
}

double cPDController::CalcTheta() const
{
	const cJoint& joint = GetJoint();
	tVector rot_axis;
	double theta = 0;
	if (UseWorldCoord())
	{
		joint.GetChildRotation(rot_axis, theta);
		tVector axis_world = joint.CalcAxisWorld();
		theta *= rot_axis.dot(axis_world);
	}
	else
	{
		joint.CalcRotation(rot_axis, theta);
	}

	return theta;
}

double cPDController::CalcVel() const
{
	const cJoint& joint = GetJoint();
	const tVector& axis_rel = joint.GetAxisRel();

	tVector joint_vel = joint.CalcJointVelRel();
	double vel = joint_vel.dot(axis_rel);
	return vel;
}

double cPDController::CalcThetaErr() const
{
	double theta = CalcTheta();
	double tar_theta = GetTargetTheta();
	double theta_err = tar_theta - theta;
	return theta_err;
}

double cPDController::CalcVelErr() const
{
	double vel = CalcVel();
	double tar_vel = GetTargetVel();
	double vel_err = tar_vel - vel;
	return vel_err;
}

double cPDController::CalcTargetTheta(double torque) const
{
	double theta = CalcTheta();
	double vel = CalcVel();

	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double tar_vel = GetTargetVel();
	double tar_theta = theta + 1 / kp * (torque - kd * (tar_vel - vel));
	return tar_theta;
}

double cPDController::CalcTargetVel(double torque) const
{
	double theta = CalcTheta();
	double vel = CalcVel();

	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double tar_theta = GetTargetTheta();
	double tar_vel = vel + 1 / kd * (torque - kp * (tar_theta - theta));
	return tar_vel;
}

double cPDController::GetTargetTheta() const
{
	return mParams[eParamTargetTheta];
}

double cPDController::GetTargetVel() const
{
	return mParams[eParamTargetVel];
}

bool cPDController::IsActive() const
{
	bool active = cController::IsActive();
	//active &= GetTorqueLimit() > 0;
	return active;
}

tVector cPDController::CalcTorque() const// no use
{
	const cJoint& joint = GetJoint();
	const tVector& axis_rel = joint.GetAxisRel();
	
	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double theta_err = CalcThetaErr();
	double vel_err = CalcVelErr();
	double t = kp * theta_err + kd * vel_err;
	tVector torque = axis_rel * t;

	return torque;
}




tVector cPDController::CalcTorque(const cJoint& joint) const{       /////////////////////////////////////attention

	tVector no_use = tVector(0,0,1,0);
	if (!joint.IsValid())
	{			
		return no_use*0;
	}
    double omiga =3 ;
    double beita = 1.5;
    double rou = 2;
    double cv= -7;                                      //cv=cv0+cv1
    double Vmax = 5;
    double bv = 0.67;
    double av=-1.668;                              //av=av0+av1+av2
    
    
    
    tVector torque;   

	if (cTerrainRLCharController::mMap.size()==0)
	{
		exit(2);
	}
	if (    cTerrainRLCharController::mMap.size()>1)
	{
		exit(1);
	}
	auto beg=cTerrainRLCharController::mMap.begin();
      Eigen::VectorXd a_thirty=Eigen::VectorXd(30);  
	//Eigen::VectorXd a_thirty;     
        a_thirty= *(beg->second);/////////////////////////get 30 num
      //  std::cout << "!"<< a_thirty.size()<<std::endl<<std::endl << a_thirty<< std::endl << "!"<< std::endl;

 Eigen::VectorXd a_twonine=Eigen::VectorXd(29);     
   for(int j=0;j<29;j++)
         {          a_twonine[j]=a_thirty[j+1];
           }
	//std::cout << "!"<< a_twonine.size()<<std::endl<<std::endl <<  a_twonine<< std::endl << "!"<< std::endl;


      // Eigen::Vector::iterator Iter=a_thirty.begin();
    //   a_twonine= a_thirty.erase( a_thirty.begin());
      //std::cout << "!"<< a_twonine.size()<<std::endl<<std::endl <<  a_twonine<< std::endl << "!"<< std::endl;
	//////////////////////////////////////????????????????????????????????????????????
                         
    double L_upbone = 0.1;	
    double L_downbone = 0.04;
    double theta = CalcTheta();
    double S_triangle = L_upbone * L_downbone*sin(theta)/2;
    double L_currmuscle = sqrt(L_upbone*L_upbone+L_downbone*L_downbone-2*L_upbone*L_downbone*cos(theta));
    double d_arm = 2*S_triangle/L_currmuscle;


    if(joint.L_muscle[0]<0)
    {
      joint.L_muscle[0]=L_currmuscle ;
    }
    double L_premuscle=joint.L_muscle[0];             //////////////attention
    double Vce = (L_currmuscle - L_premuscle)/0.0017;
    double FL = exp (-pow(abs((pow(L_currmuscle,beita)-1)/omiga),rou));
    double FV;
    if (Vce>0)
    {
       FV = (bv-(av*Vce))/(bv+Vce);
    }
    else
    {
       FV = (Vmax-Vce)/(Vmax+(cv*Vce));
    }
  
    torque=a_twonine*FL*FV*d_arm;
    joint.L_muscle[0]=L_currmuscle ;
   
	 
	return torque;
}


int cPDController::GetJointID() const
{
	return static_cast<int>(mParams[eParamJointID]);
}
