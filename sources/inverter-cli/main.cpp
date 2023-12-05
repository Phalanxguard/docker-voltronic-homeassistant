// Lightweight program to take the sensor data from a Voltronic Axpert, Mppsolar PIP, Voltacon, Effekta, and other branded OEM Inverters and send it to a MQTT server for ingestion...
// Adapted from "Maio's" C application here: https://skyboo.net/2017/03/monitoring-voltronic-power-axpert-mex-inverter-under-linux/
//
// Please feel free to adapt this code and add more parameters -- See the following forum for a breakdown on the RS323 protocol: http://forums.aeva.asn.au/viewtopic.php?t=4332
// ------------------------------------------------------------------------

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include "main.h"
#include "tools.h"
#include "inputparser.h"

#include <pthread.h>
#include <signal.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>


bool debugFlag = false;
bool runOnce = false;

cInverter *ups = NULL;

atomic_bool ups_status_changed(false);
atomic_bool ups_qmod_changed(false);
atomic_bool ups_qpiri_changed(false);
atomic_bool ups_qpigs_changed(false);
atomic_bool ups_qpigs2_changed(false);
atomic_bool ups_qpiws_changed(false);
atomic_bool ups_cmd_executed(false);


// ---------------------------------------
// Global configs read from 'inverter.conf'

string devicename;
int runinterval;
float ampfactor;
float wattfactor;
int qpiri, qpiws, qmod, qpigs, qpigs2;

// ---------------------------------------

void attemptAddSetting(int *addTo, string addFrom) {
    try {
        *addTo = stof(addFrom);
    } catch (exception e) {
        cout << e.what() << '\n';
        cout << "There's probably a string in the settings file where an int should be.\n";
    }
}

void attemptAddSetting(float *addTo, string addFrom) {
    try {
        *addTo = stof(addFrom);
    } catch (exception e) {
        cout << e.what() << '\n';
        cout << "There's probably a string in the settings file where a floating point should be.\n";
    }
}

void getSettingsFile(string filename) {

    try {
        string fileline, linepart1, linepart2;
        ifstream infile;
        infile.open(filename);

        while(!infile.eof()) {
            getline(infile, fileline);
            size_t firstpos = fileline.find("#");

            if(firstpos != 0 && fileline.length() != 0) {    // Ignore lines starting with # (comment lines)
                size_t delimiter = fileline.find("=");
                linepart1 = fileline.substr(0, delimiter);
                linepart2 = fileline.substr(delimiter+1, string::npos - delimiter);

                if(linepart1 == "device")
                    devicename = linepart2;
                else if(linepart1 == "run_interval")
                    attemptAddSetting(&runinterval, linepart2);
                else if(linepart1 == "amperage_factor")
                    attemptAddSetting(&ampfactor, linepart2);
                else if(linepart1 == "watt_factor")
                    attemptAddSetting(&wattfactor, linepart2);
                else if(linepart1 == "qpiri")
                    attemptAddSetting(&qpiri, linepart2);
                else if(linepart1 == "qpiws")
                    attemptAddSetting(&qpiws, linepart2);
                else if(linepart1 == "qmod")
                    attemptAddSetting(&qmod, linepart2);
                else if(linepart1 == "qpigs")
                    attemptAddSetting(&qpigs, linepart2);
				else if(linepart1 == "qpigs2")
                    attemptAddSetting(&qpigs2, linepart2);
                else
                    continue;
            }
        }
        infile.close();
    } catch (...) {
        cout << "Settings could not be read properly...\n";
    }
}

int main(int argc, char* argv[]) {

    // Reply1
    float voltage_grid;
    float freq_grid;
    float voltage_out;
    float freq_out;
    int load_va;
    int load_watt;
    int load_percent;
    int voltage_bus;
    float voltage_batt_1;
    int batt_charge_current;
    int batt_capacity;
    int temp_heatsink;
    float pv_input_current_1;
    float pv_input_voltage_1;
    float pv_input_watts;
    float pv_input_watthour;
    float load_watthour = 0;
    float scc_voltage;
    int batt_discharge_current;
    char device_status_1[9];

    // Reply2
    float grid_voltage_rating;
    float grid_current_rating;
    float out_voltage_rating;
    float out_freq_rating;
    float out_current_rating;
    int out_va_rating;
    int out_watt_rating;
    float batt_rating;
    float batt_recharge_voltage;
    float batt_under_voltage;
    float batt_bulk_voltage;
    float batt_float_voltage;
    int batt_type;
    int max_grid_charge_current;
    int max_charge_current;
    int in_voltage_range;
    int out_source_priority;
    int charger_source_priority;
    int machine_type;
    int topology;
    int out_mode;
    float batt_redischarge_voltage;
	
	// Reply3
    float pv_input_current_2;
	float pv_input_voltage_2;
	float voltage_batt_2;
	float pv_input_watts_2;
	char device_status_2[9];		//Charging status on/off (SCC2) b1, Charging status on/off (SCC3)b2, Reserved b3 to b8
	float ac_charging_current;
	float ac_charging_power;
	float pv_input_current_3;
	float pv_input_voltage_3;
	float voltage_batt_3;
	float pv_input_watts_3;
	float pv_input_total_watts;
	
    

    // Get command flag settings from the arguments (if any)
    InputParser cmdArgs(argc, argv);
    const string &rawcmd = cmdArgs.getCmdOption("-r");

    if(cmdArgs.cmdOptionExists("-h") || cmdArgs.cmdOptionExists("--help")) {
        return print_help();
    }
    if(cmdArgs.cmdOptionExists("-d")) {
        debugFlag = true;
    }
    if(cmdArgs.cmdOptionExists("-1") || cmdArgs.cmdOptionExists("--run-once")) {
        runOnce = true;
    }
    lprintf("INVERTER: Debug set");

    // Get the rest of the settings from the conf file
    if( access( "./inverter.conf", F_OK ) != -1 ) { // file exists
        getSettingsFile("./inverter.conf");
    } else { // file doesn't exist
        getSettingsFile("/etc/inverter/inverter.conf");
    }

    bool ups_status_changed(false);
    ups = new cInverter(devicename,qpiri,qpiws,qmod,qpigs,qpigs2);

    // Logic to send 'raw commands' to the inverter..
    if (!rawcmd.empty()) {
        ups->ExecuteCmd(rawcmd);
        // We're piggybacking off the qpri status response...
        printf("Reply:  %s\n", ups->GetQpiriStatus()->c_str());
        exit(0);
    } else {
        ups->runMultiThread();
    }

    while (true) {
        if (ups_status_changed) {
            int mode = ups->GetMode();

            if (mode)
                lprintf("INVERTER: Mode Currently set to: %d", mode);

            ups_status_changed = false;
        }

        if (ups_qmod_changed && ups_qpiri_changed && ups_qpigs_changed && ups_qpigs2_changed) {

            ups_qmod_changed = false;
            ups_qpiri_changed = false;
            ups_qpigs_changed = false;
			ups_qpigs2_changed = false;

            int mode = ups->GetMode();
            string *reply1   = ups->GetQpigsStatus();
            string *reply2   = ups->GetQpiriStatus();
			string *reply3   = ups->GetQpigs2Status();
            string *warnings = ups->GetWarnings();

            if (reply1 && reply2 && reply3 && warnings) {

                // Parse and display values
                sscanf(reply1->c_str(), "%f %f %f %f %d %d %d %d %f %d %d %d %f %f %f %d %s", &voltage_grid, &freq_grid, &voltage_out, &freq_out, &load_va, &load_watt, &load_percent, &voltage_bus, &voltage_batt_1, &batt_charge_current, &batt_capacity, &temp_heatsink, &pv_input_current_1, &pv_input_voltage_1, &scc_voltage, &batt_discharge_current, &device_status_1);
                sscanf(reply2->c_str(), "%f %f %f %f %f %d %d %f %f %f %f %f %d %d %d %d %d %d - %d %d %d %f", &grid_voltage_rating, &grid_current_rating, &out_voltage_rating, &out_freq_rating, &out_current_rating, &out_va_rating, &out_watt_rating, &batt_rating, &batt_recharge_voltage, &batt_under_voltage, &batt_bulk_voltage, &batt_float_voltage, &batt_type, &max_grid_charge_current, &max_charge_current, &in_voltage_range, &out_source_priority, &charger_source_priority, &machine_type, &topology, &out_mode, &batt_redischarge_voltage);
				sscanf(reply3->c_str(), "%f %f %f %f %s %f %f %f %f %f %f %f", &pv_input_current_2, &pv_input_voltage_2, &voltage_batt_2, &pv_input_watts_2, &device_status_2, &ac_charging_current, &ac_charging_power, &pv_input_current_3, &pv_input_voltage_3, &voltage_batt_3, &pv_input_watts_3, &pv_input_total_watts);





                // There appears to be a discrepancy in actual DMM measured current vs what the meter is
                // telling me it's getting, so lets add a variable we can multiply/divide by to adjust if
                // needed.  This should be set in the config so it can be changed without program recompile.
                if (debugFlag) {
                    printf("INVERTER: ampfactor from config is %.2f\n", ampfactor);
                    printf("INVERTER: wattfactor from config is %.2f\n", wattfactor);
                }

                pv_input_current_1 = pv_input_current_1 * ampfactor;
				pv_input_current_2 = pv_input_current_2 * ampfactor;
				pv_input_current_3 = pv_input_current_3 * ampfactor;
                // It appears on further inspection of the documentation, that the input current is actually
                // current that is going out to the battery at battery voltage (NOT at PV voltage).  This
                // would explain the larger discrepancy we saw before.

                pv_input_watts = (pv_input_voltage_1 * pv_input_current_1) * wattfactor;

                // Calculate watt-hours generated per run interval period (given as program argument)
                pv_input_watthour = pv_input_watts / (3600 / runinterval);
                load_watthour = (float)load_watt / (3600 / runinterval);

                // Print as JSON (output is expected to be parsed by another tool...)
                printf("{\n");

                printf("  \"Inverter_mode\":%d,\n", mode);
                printf("  \"AC_grid_voltage\":%.1f,\n", voltage_grid);
                printf("  \"AC_grid_frequency\":%.1f,\n", freq_grid);
                printf("  \"AC_out_voltage\":%.1f,\n", voltage_out);
                printf("  \"AC_out_frequency\":%.1f,\n", freq_out);
                printf("  \"PV_in_voltage_1\":%.1f,\n", pv_input_voltage_1);
                printf("  \"PV_in_current_1\":%.1f,\n", pv_input_current_1);
                printf("  \"PV_in_watts_1\":%.1f,\n", pv_input_watts);
                printf("  \"PV_in_watthour\":%.4f,\n", pv_input_watthour);
                printf("  \"SCC_voltage\":%.4f,\n", scc_voltage);
                printf("  \"Load_pct\":%d,\n", load_percent);
                printf("  \"Load_watt\":%d,\n", load_watt);
                printf("  \"Load_watthour\":%.4f,\n", load_watthour);
                printf("  \"Load_va\":%d,\n", load_va);
                printf("  \"Bus_voltage\":%d,\n", voltage_bus);
                printf("  \"Heatsink_temperature\":%d,\n", temp_heatsink);
                printf("  \"Battery_capacity\":%d,\n", batt_capacity);
                printf("  \"Battery_voltage_1\":%.2f,\n", voltage_batt_1);
                printf("  \"Battery_charge_current\":%d,\n", batt_charge_current);
                printf("  \"Battery_discharge_current\":%d,\n", batt_discharge_current);
                printf("  \"Load_status_on\":%c,\n", device_status_1[3]);
                printf("  \"SCC1_charge_on\":%c,\n", device_status_1[6]);
                printf("  \"AC_charge_on\":%c,\n", device_status_1[7]);
                printf("  \"Battery_recharge_voltage\":%.1f,\n", batt_recharge_voltage);
                printf("  \"Battery_under_voltage\":%.1f,\n", batt_under_voltage);
                printf("  \"Battery_bulk_voltage\":%.1f,\n", batt_bulk_voltage);
                printf("  \"Battery_float_voltage\":%.1f,\n", batt_float_voltage);
                printf("  \"Max_grid_charge_current\":%d,\n", max_grid_charge_current);
                printf("  \"Max_charge_current\":%d,\n", max_charge_current);
                printf("  \"Out_source_priority\":%d,\n", out_source_priority);
                printf("  \"Charger_source_priority\":%d,\n", charger_source_priority);
                printf("  \"Battery_redischarge_voltage\":%.1f,\n", batt_redischarge_voltage);
                printf("  \"Warnings\":\"%s\"\n", warnings->c_str());
				printf("  \"PV_in_current_2\":%.1f,\n", pv_input_current_2);
				printf("  \"PV_in_voltage_2\":%.1f,\n", pv_input_voltage_2);
				printf("  \"Battery_voltage_2\":%.2f,\n", voltage_batt_2);
				printf("  \"PV_in_watts_2\":%.1f,\n", pv_input_watts_2);
				printf("  \"SCC2_charge_on\":%c,\n", device_status_2[0]);
				printf("  \"SCC3_charge_on\":%c,\n", device_status_2[1]);
				printf("  \"ac_charge_current\":%d,\n", ac_charging_current);
				printf("  \"ac_charge_power\":%d,\n", ac_charging_power);
				printf("  \"PV_in_current_3\":%.1f,\n", pv_input_current_3);
				printf("  \"PV_in_voltage_3\":%.1f,\n", pv_input_voltage_3);
				printf("  \"Battery_voltage_3\":%.2f,\n", voltage_batt_3);
				printf("  \"PV_in_watts_3\":%.1f,\n", pv_input_watts_3);
				printf("  \"PV_in_watts_total\":%.1f,\n", pv_input_total_watts);
                printf("}\n");
				


                // Delete reply string so we can update with new data when polled again...
                delete reply1;
                delete reply2;
				delete reply3;

                if(runOnce) {
                    // Do once and exit instead of loop endlessly
                    lprintf("INVERTER: All queries complete, exiting loop.");
                    exit(0);
                }
            }
        }

        sleep(1);
    }

    if (ups)
        delete ups;
    return 0;
}
