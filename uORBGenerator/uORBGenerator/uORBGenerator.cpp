// uORBGenerator.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

std::ifstream input("orb_list.txt");
std::ifstream in_orb_multi("orb_multi.txt");
std::ofstream output("uORBHelper.cpp");
std::ofstream output_header("uORBHelper.h");

#define MAX_ORB_NUM  200


int main()
{
	std::string orb_names[MAX_ORB_NUM], 
				orb_types[MAX_ORB_NUM], 
				orb_public[MAX_ORB_NUM], 
				orb_multi[MAX_ORB_NUM];

	int orb_count = 0;
	int orb_multi_count = 0;

	while (std::getline(input, orb_names[orb_count]))
	{
			
		orb_types[orb_count] = orb_names[orb_count] + "_s";
		orb_public[orb_count] = "ORB_" + orb_names[orb_count] + "_public";

		++orb_count;
	}

	while (std::getline(in_orb_multi, orb_multi[orb_multi_count]))
	{
		++orb_multi_count;
	}


	output << "#include <uORB/uORB.h>" << std::endl;
	output << "#include <cstring>" << std::endl;
	output << "#include <StartUP.h>" << std::endl << std::endl;
	output << "#include \"uORBHelper.h\"" << std::endl;
	output << std::endl;
	output << std::endl;
	for (int i = 0; i < orb_count; ++i)
	{
		output << "#include <uORB/topics/" << orb_names[i] << ".h>" << std::endl;
	}
	output << std::endl;
	output << std::endl;
	output << std::endl;
	output << "#pragma default_variable_attributes= @ \".ccmram\"" << std::endl;
	output << std::endl;
	output << std::endl;

	for (int i = 0; i < orb_count; ++i)
	{
		bool found_multi = false;
		for (int multi_index = 0; multi_index < orb_multi_count; ++multi_index)
		{
			if (orb_multi[multi_index] == orb_names[i])
			{
				found_multi = true;
				break;
			}
		}

		if (found_multi)
		{
			output << "  " << orb_types[i] << "\t\t\t" << orb_public[i] << "[ORB_MULTI_MAX_INSTANCES];" << std::endl;
		}
		else
			output << "  " << orb_types[i] << "\t\t\t" << orb_public[i] << ";" << std::endl;
	}
	
	output << std::endl;
	output << std::endl;
	output << "#pragma default_variable_attributes = " << std::endl;
	output << std::endl;
	output << std::endl;
	output << std::endl;

	output << std::endl;
	output << std::endl;
	output << "void orb_helper_init(void)" << std::endl;
	output << "{" << std::endl;

	for (int i = 0; i < orb_count; ++i)
	{
		bool found_multi = false;
		for (int multi_index = 0; multi_index < orb_multi_count; ++multi_index)
		{
			if (orb_multi[multi_index] == orb_names[i])
			{
				found_multi = true;
				break;
			}
		}

		if (found_multi)
		{
			output << "  " << "std::memset(&" << orb_public[i] << ", 0, ORB_MULTI_MAX_INSTANCES*sizeof(" << orb_types[i] << "));" << std::endl;
		}
		else
			output << "  " << "std::memset(&" << orb_public[i] << ", 0, sizeof(" << orb_types[i] << "));" << std::endl;
	}

	


	output << "}" << std::endl;
	output << std::endl;
	output << std::endl;

	output << "void orb_set_in_os(void)" << std::endl;
	output << "{" << std::endl;
	output << "    orb_in_os = true;" << std::endl;
	output << "}" << std::endl;

	output << std::endl;
	output << std::endl;

	output << "bool is_orb_multi(int serial)" << std::endl;
	output << "{" << std::endl;
	output << "    for (int i = 0; i<MULTI_ORB_NUM; ++i)" << std::endl;
	output << "	   {" << std::endl;
	output << "	       if (serial == (int)orb_multi_list[i])" << std::endl;
	output << "		       return true;" << std::endl;
	output << "	   }" << std::endl;
	output << "" << std::endl;
	output << "	   return false;" << std::endl;
	output << "}" << std::endl;

	output << std::endl;
	output << std::endl;

	output << "int get_priority(int instance)" << std::endl;
	output << "{" << std::endl;
	output << "		if (instance == 0)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_MIN;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 1)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_VERY_LOW;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 2)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_LOW;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 3)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_DEFAULT;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 4)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_HIGH;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 5)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_VERY_HIGH;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (instance == 6)" << std::endl;
	output << "		{" << std::endl;
	output << "			return ORB_PRIO_MAX;" << std::endl;
	output << "		}" << std::endl;
	output << "		else" << std::endl;
	output << "			return -1;" << std::endl;
	output << "}" << std::endl;

	output << std::endl;
	output << std::endl;


	output << "int get_orb_instance_according_to_priority(int priority)" << std::endl;
	output << "{" << std::endl;
	output << "		if (ORB_PRIO_MIN == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 0;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_VERY_LOW == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 1;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_LOW == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 2;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_DEFAULT == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 3;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_HIGH == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 4;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_VERY_HIGH == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 5;" << std::endl;
	output << "		}" << std::endl;
	output << "		else if (ORB_PRIO_MAX == priority)" << std::endl;
	output << "		{" << std::endl;
	output << "			return 6;" << std::endl;
	output << "		}" << std::endl;
	output << "		else" << std::endl;
	output << "			return -1;" << std::endl;
	output << "}" << std::endl;
	

	output << "void get_orb_name(ORB_serial serial, char * name)" << std::endl;
	output << "{" << std::endl;

	for (int i = 0; i < orb_count;++i)
	{
		if (i==0)
			output << "  if (serial == ORB_" << orb_names[i] << ")" << std::endl;
		else
			output << "  else if (serial == ORB_" << orb_names[i] << ")" << std::endl;
		output << "  {"<<std::endl;
		output << "      std::strcpy(name, (ORB_ID(" << orb_names[i] << "))->o_name);" << std::endl;
		output << "  }" << std::endl;
	}
	output << "}" << std::endl;
	output << std::endl;
	output << std::endl;

	output << "int  get_orb_serial(const char * name)" << std::endl;
	output << "{" << std::endl;
	for (int i = 0; i < orb_count; ++i)
	{
		if (i == 0)
			output << "  if (0 == std::strcmp(name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
		else
			output << "  else if (0 == std::strcmp(name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
		output << "  {" << std::endl;
		output << "      return ORB_" << orb_names[i] << ";" << std::endl;
		output << "  }" << std::endl;
	}
	output << "else" << std::endl;
	output << "    return -1;" << std::endl;
	output << "}" << std::endl;
	output << std::endl;
	output << std::endl;


	output << "orb_id_t  get_orb_according_to_serial(int serial)" << std::endl;
	output << "{" << std::endl;
	output << "  char orb_name[50];" << std::endl;
	output << "  get_orb_name((ORB_serial)serial,orb_name);" << std::endl;
	output << std::endl;
	output << std::endl;
	for (int i = 0; i < orb_count; ++i)
	{
		if (i == 0)
			output << "  if (0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
		else
			output << "  else if (0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
		output << "  {" << std::endl;
		output << "      return ORB_ID(" << orb_names[i] << ");" << std::endl;
		output << "  }" << std::endl;
	}
	output << "else" << std::endl;
	output << "    return nullptr;" << std::endl;
	output << "}" << std::endl;
	output << std::endl;
	output << std::endl;

	output << "void  *get_orb_public_according_to_serial_and_instance(int serial, int instance)" << std::endl;
	output << "{" << std::endl;
	output << "  char orb_name[50];" << std::endl;
	output << "  get_orb_name((ORB_serial)serial,orb_name);" << std::endl;
	output << std::endl;
	output << "  bool is_multi = is_orb_multi(serial);" << std::endl;
	output << std::endl;
	output << "  if(!is_orb_multi(serial) && instance >0)" << std::endl;
	output << "      return nullptr;" << std::endl;
	output << std::endl;
	for (int i = 0; i < orb_count; ++i)
	{

		bool found_multi = false;
		for (int multi_index = 0; multi_index < orb_multi_count; ++multi_index)
		{
			if (orb_multi[multi_index] == orb_names[i])
			{
				found_multi = true;
				break;
			}
		}

		if (!found_multi)
		{

			if (i == 0)
				output << "  if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
			else
				output << "  else if (!is_multi && instance ==0 && 0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
			output << "  {" << std::endl;
			output << "      return (void*)&(ORB_" << orb_names[i] << "_public);" << std::endl;
			output << "  }" << std::endl;
		}
		else
		{
			if (i == 0)
				output << "  if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
			else
				output << "  else if (is_multi && 0 == std::strcmp(orb_name, (ORB_ID(" << orb_names[i] << "))->o_name))" << std::endl;
			
			output << "  {" << std::endl;
			output << "        return (void*)&(ORB_" << orb_names[i] << "_public[instance]);" << std::endl;
			output << "  }" << std::endl;
		}
	}
	output << "else" << std::endl;
	output << "    return nullptr;" << std::endl;
	output << "}" << std::endl;



	//Header

	output_header << "#ifndef __UORB_HELPER_H" << std::endl;
	output_header << "#define __UORB_HELPER_H" << std::endl;

	output_header << std::endl;
	output_header << std::endl;

	output_header << "#include <uORB/uORB.h>" << std::endl;
	output_header << "#include <cmsis_os.h>" << std::endl;
	
	output_header << std::endl;
	output_header << std::endl;

	output_header << "enum ORB_serial" << std::endl;
	output_header << "{" << std::endl;
	for (int i = 0; i < orb_count;++i)
	{
		output_header << "    ORB_" << orb_names[i] << " = " << i << "," << std::endl;
	}
	output_header << "    total_uorb_num" << std::endl;
	output_header << "};" << std::endl;

	output_header << "struct ORBData{" << std::endl;
	output_header << "    void                *data = nullptr;" << std::endl;
	output_header << "    ORB_PRIO            priority;" << std::endl;
	output_header << "    int                 serial;" << std::endl;
	output_header << "    uint64_t            interval;" << std::endl;
	output_header << "    xQueueHandle        queue;" << std::endl;
	output_header << "    bool                published;" << std::endl;
	output_header << "};" << std::endl;
	
	output_header << std::endl;
	output_header << std::endl;
	output_header << "const int MULTI_ORB_NUM = " << orb_multi_count << ";" << std::endl;
	output_header << std::endl;
	output_header << std::endl;

	output_header << "const ORB_serial orb_multi_list[MULTI_ORB_NUM]=" << std::endl;
	output_header << "{" << std::endl;
	for (int i = 0; i < orb_multi_count; ++i)
	{
		if (i != (orb_multi_count - 1))
			output_header << "    ORB_" << orb_multi[i] << "," << std::endl;
		else
			output_header << "    ORB_" << orb_multi[i] << "," << std::endl;
	}
	output_header << "};" << std::endl;
	output_header << std::endl;
	output_header << std::endl;
	output_header << "#define general_type_ptr        parameter_update_s*" << std::endl;
	output_header << std::endl;
	output_header << std::endl;


	output_header << "extern ORBData          *orb_data[total_uorb_num];" << std::endl;

	output_header << "extern void             get_orb_name(ORB_serial serial, char * name);" << std::endl;
	output_header << "extern int              get_orb_serial(const char * name);" << std::endl;
	output_header << "extern int              get_orb_instance_according_to_priority(int priority);" << std::endl;
	output_header << "extern int              get_priority(int instance);" << std::endl;

	output_header << "extern bool             orb_in_os;" << std::endl;

	output_header << "extern void             orb_helper_init(void);" << std::endl;

	output_header << "extern void             orb_set_in_os(void);" << std::endl;
	output_header << "extern bool             is_orb_multi(int serial);" << std::endl;
	output_header << "extern orb_id_t         get_orb_according_to_serial(int serial);" << std::endl;
	output_header << "extern void             *get_orb_public_according_to_serial_and_instance(int serial, int instance);" << std::endl;

	output_header << std::endl;
	output_header << std::endl;
	output_header << "#endif" << std::endl;

	return 0;
}

