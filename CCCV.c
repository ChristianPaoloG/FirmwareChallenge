double sensed_output, control_signal;
double setpoint;
double Kp; //proportional gain
double Ki; //integral gain
int T; //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
long max_control_c;
long min_control_c;
long max_control_v;
long min_control_c;

long current_feedback;
long current_reference;
long voltage_feedback;
long voltage_reference;
long minimum_current


enum class States : uint8_t
{         
    idle,          
    constantCurrent,              
    constantVoltage      
};

void Initialization(void){
DDRA &= (1 << PINA1);
DDRA &= (1 << PINA2);
DDRA &= (1 << PINA3);
}
void control_routine(void){
 current_feedback = PINA & (1 << PINA1);
 voltage_feedback = PINA & (1 << PINA2);
 enable_command = PINA & (1 << PINA3);

}
void main_state_machine(void){
unsigned long current_time = millis(); //returns the number of milliseconds passed start of the program
int delta_time = current_time - last_time; //delta time interval
States newState = state;

switch(state){
	case idle:{
		if (enable_command = true){
			state = constantCurrent;
	}
	break;
	
	case constantCurrent:{
		if (voltage_feedback = voltage_reference){
			state = constantVoltage;
		}
		if (delta_time >= T){
			double error = current_reference - current_feedback;
			total_error += error; 
			if (total_error >= max_control) total_error = max_control;
			else if (total_error <= min_control) total_error = min_control;


			control_signal = Kp*error + (Ki*T)*total_error
			if (control_signal >= max_control) control_signal = max_control;
			else if (control_signal <= min_control) control_signal = min_control;
		
		}
		break;
		
	case constantVoltage:{
		if (current_feedback = minimum_current){
			state = idle;
		}
		if (delta_time >= T){
			double error = voltage_reference - voltage_feedback;
			total_error += error; 
			if (total_error >= max_control) total_error = max_control;
			else if (total_error <= min_control) total_error = min_control;


			control_signal = Kp*error + (Ki*T)*total_error
			if (control_signal >= max_control) control_signal = max_control;
			else if (control_signal <= min_control) control_signal = min_control;
		
		}
	
	last_error = error;
	last_time = current_time;
}

void main(void){
Initialization();
PieVectTable.EPWM1_INT = &control_routine;
while(true){
main_state_machine();
}