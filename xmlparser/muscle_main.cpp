#include <tinyxml.h>
#include <Eigen/Core>
#include <map>
#include <vector>
#include <fstream>
#define LARGE_VALUE 1E6
#define REVOLUTE_JOINT_LIMIT 0.05
#define PROXIMAR_JOINT_LIMIT 1.0
namespace Eigen
{
	typedef Matrix<double,1,1> Vector1d;
};
struct UserConstant
{
	UserConstant(double _f0,double _lm,double _lt,double angle)
		:f0(_f0),lm(_lm),lt(_lt),pen_angle(angle)
	{}
	double f0;
	double lm;
	double lt;
	double pen_angle;
	
};
struct MayaConstant
{
	MayaConstant(std::string name)
		:mName(name)
	{}
	void AddAnchor(std::string body,Eigen::Vector3d glob_pos)
	{
		mAnchors.push_back(std::make_pair(body,glob_pos));
	}
	std::string mName;
	std::vector<std::pair<std::string,Eigen::Vector3d>> mAnchors;
};
std::string toString(const Eigen::Vector3d& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;

}
std::string toString(const Eigen::VectorXd& v)
{
	std::string ret ="";
	for(int i=0;i<v.rows();i++){
		ret += std::to_string(v[i]);
		ret += " ";
	}
	return ret;	
}
void ReadMayaConstant(std::vector<MayaConstant>& mc,std::string path)
{
	std::ifstream ifs(path);
	if(!(ifs.is_open()))
	{
		std::cout<<"Can't read file "<<path<<std::endl;
		return;
	}
	std::string str;
	std::string index;
	std::stringstream ss;

	std::string name;
	double x,y,z;
	while(!ifs.eof())
	{
		str.clear();
		index.clear();
		ss.clear();

		std::getline(ifs,str);
		ss.str(str);
		ss>>index;
		if(index=="unit")
		{
			ss>>name;
			// std::cout<<"unit "<<name<<std::endl;
			std::cout<<name<<std::endl;
			mc.push_back(MayaConstant(name));
		}
		else if(index=="attach")
		{
			ss>>name;

			ss>>x>>y>>z;
			// std::cout<<"attach "<<name<<" "<<-x<<" "<<y<<" "<<z<<std::endl;
			mc.back().AddAnchor(name,Eigen::Vector3d(x,y,z));
		}
	}

	ifs.close();
}
int main(int argc,char** argv)
{
	std::map<std::string,UserConstant> ucs;
	std::vector<MayaConstant> mcs;
	ReadMayaConstant(mcs,argv[1]);
	// exit(0);
	for(int i =0;i<mcs.size();i++)
		ucs.insert(std::make_pair(mcs[i].mName,UserConstant(2000.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Adductor_Longus1",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Bicep_Femoris_Longus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Bicep_Femoris_Short",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Extensor_Digitorum_Longus1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Extensor_Hallucis_Longus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Flexor_Digitorum_Longus2",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Flexor_Hallucis",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gastrocnemius_Lateral_Head",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus2",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Maximus4",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gluteus_Medius1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Gracilis",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Obturator_Externus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Obturator_Internus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Peroneus_Longus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Plantaris",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Psoas_Major2",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Rectus_Femoris",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Sartorius",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Semimembranosus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Semitendinosus",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Soleus1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Tensor_Fascia_Lata2",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Tibialis_Anterior",UserConstant(300.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Intermedius1",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Lateralis",UserConstant(500.0,1.0,0.2,0.0)));
	// ucs.insert(std::make_pair("L_Vastus_Medialis1",UserConstant(500.0,1.0,0.2,0.0)));

	TiXmlDocument doc;
	TiXmlElement* muscle_elem = new TiXmlElement("Muscle");
	doc.LinkEndChild(muscle_elem);

	for(int i =0;i<mcs.size();i++)
	{
		TiXmlElement* unit_elem = new TiXmlElement("Unit");
		std::cout<<mcs[i].mName<<std::endl;
		auto& uc = ucs.at(mcs[i].mName);

		unit_elem->SetAttribute("name",mcs[i].mName);
		unit_elem->SetAttribute("f0",std::to_string(uc.f0));
		unit_elem->SetAttribute("lm",std::to_string(uc.lm));
		unit_elem->SetAttribute("lt",std::to_string(uc.lt));
		unit_elem->SetAttribute("pen_angle",std::to_string(uc.pen_angle));

		for(int j =0;j<mcs[i].mAnchors.size();j++)
		{
			TiXmlElement* waypoint_elem = new TiXmlElement("Waypoint");
			waypoint_elem->SetAttribute("body",mcs[i].mAnchors[j].first);
			waypoint_elem->SetAttribute("p",toString(mcs[i].mAnchors[j].second));
			unit_elem->LinkEndChild(waypoint_elem);	
		}
		muscle_elem->LinkEndChild(unit_elem);
		
		
	}
	TiXmlPrinter printer;
	printer.SetIndent( "\n" );

	doc.Accept( &printer );
	doc.SaveFile(argv[2]);
	
	return 0;
}