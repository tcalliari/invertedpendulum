
#include <PID_v1.h>
#include <Encoder.h>


//DEFINIÇÃO DE VARIÁVEIS

//PINOS
    byte pin_photoint=13;
    byte pin_pwm=5;
    byte pin_left=7; //direção de rotação do motor
    byte pin_right=6;
    byte pin_lin_enc1=2;  //todos os encoders precisam de interrupção
    byte pin_lin_enc2=3;  //todos os encoders precisam de interrupção
    byte pin_rot_enc1=20; //todos os encoders precisam de interrupção
    byte pin_rot_enc2=21; //todos os encoders precisam de interrupção
    byte pin_led=14;
   
  //Speeds
    byte setup_drive_speed=150; //velocidade do motor no setup inicial, PWM, 0 a 255
    byte setup_zero_speed=90;  //velocidade que o motor não é capaz de movimentar o carrinho

//LIMITES DE CURSO
    int limit_lin_min=400;    //posição mínima, para a esquerda, lado do sensor de posição
    int limit_lin_max=5800;   //posição máxima, para a direita
    double limit_angle=129;   //limite de ânuglo onde o sistema faz o controle do pêndulo, se for acima o sistema não consegue efetuar correções a tempo
    
 
    
    byte rot_deadband=0; 
    int rot_offset=0;   		//offset para o ângulo
    double Err_angle;			//erro do ângulo
    double lastErr_angle=0; 	//ultimo erro lido
    double timer;
    double lin_speed;			//velocidade sensor linear
    double rot_speed;			//velocidade sensor rotacional
    const int array_length=50;
    double lin_position_array[array_length];
    double angle_array[array_length];
    double timer_array[array_length];
    double angle;				//angulo lido pelo sensor
    double lin_position;		//posição linear atual
    double lin_Setpoint_outer=2500;	//posição inicial desejada para a base do pendulo
    double lin_kd_avg;  		

   double timer_data[8000];
   double rot_Setpoint_data[8000];
   double lin_position_data[8000];
   double angle_data[8000];
   double Output_data[8000];
   
	int leitura_serial = -1;
	int leitura = 0;
	bool comando = false;


//Criação de objetos para o encoder
  Encoder linEnc(pin_lin_enc1,pin_lin_enc2);                
  Encoder rotEnc(pin_rot_enc1,pin_rot_enc2);  
 //Inicialização do PID
    double Output, rot_Setpoint=0, lin_Setpoint=3100; //posição desejada da base
    double rot_kp=0, rot_ki=0, rot_kd=0.00;  
    double lin_kp=0, lin_ki=0.0, lin_kd=0;  
    PID rot_Controller(&angle,&Output,&rot_Setpoint,rot_kp,rot_ki,rot_kd,DIRECT);
    PID lin_Controller(&lin_position,&rot_Setpoint,&lin_Setpoint,lin_kp,lin_ki,lin_kd,DIRECT); 



//INICIO PROCEDIMENTO SETUP
void setup() {

       Serial.begin(115200);		//velocidade comunicação serial  
       while (leitura_serial != 88) {
			while (Serial.available()==0){};   		
			leitura_serial = Serial.read();
	   }
		//X - Modo PID
		Serial.println('X');	   
		while (leitura_serial != 73) {
			while (Serial.available()==0){     
			}
			leitura_serial = Serial.read(); // aguarda receber o I   
		}
		Serial.println(leitura_serial);  
    
		pinMode(pin_photoint,INPUT);	//atribui entradas e saidas
		pinMode(pin_led,OUTPUT); 
	   
		      
		//calibração dos sensores, inicialização da planta
       linEnc.write(12000); //para funcionar inicialmente o sensor linear tem que ter um valor de inicialização maior que o total de posições que ele pode ler
       rotEnc.write(-2400); //pendulo totalmente para baixo
       delay(500);
       
     
      

      

		//movimento da base em direção ao acoplador otico
        go_right(180);		//da um impulso inicial no motor para vencer o atrito estático
        delay(10);
		//movimenta a base em velocidade normal até o sensor detectar a base          
        while (linEnc.read()!=0){
            go_right(setup_drive_speed);
            byte photoint=digitalRead(pin_photoint);
            if (photoint==1){ 
              break;
              }
            }
              
              go_stop();          //para o movimento
              delay(300);
              linEnc.write(0);    //zera a contagem do encoder linear
              
    
					//movimento até o meio da estrutura
                    go_left(180);
                    delay(10);
         while (linEnc.read()<lin_Setpoint){    
                go_left(setup_drive_speed);
              }
              
              go_stop();
              delay(500);           

	//Configurações para o PID
     rot_Controller.SetMode(AUTOMATIC);
     rot_Controller.SetOutputLimits(-255,255); 
     rot_Controller.SetSampleTime(2);
     lin_Controller.SetMode(AUTOMATIC);
     lin_Controller.SetOutputLimits(-300,300); 
     lin_Controller.SetSampleTime(2);
     

delay(300);
}


void loop() {
		
			//Posicionando a leitura da posição linear no array
			for (int x=array_length; x>1; x--){
			  lin_position_array[x-1]=lin_position_array[x-2];
			}
			//Posicionando a leitura do ngulo no array
			for (int x=array_length; x>1; x--){
			  angle_array[x-1]=angle_array[x-2];
			}
		 
			//Posicionando a leitura do timer no array   
			for (int x=array_length; x>1; x--){
			  timer_array[x-1]=timer_array[x-2];
			}
			
			//lendo os valores atuais para o timer, angulo, e posição linear e salvando na primeira posição dos arrays
			timer=millis();
			timer_array[0]=timer;
			angle=rotEnc.read();
			angle_array[0]=angle;
			lin_position=linEnc.read();
			lin_position_array[0]=lin_position;
			//calculo da velocidade linear
			lin_speed=(lin_position_array[0]-lin_position_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);
			rot_speed=(angle_array[0]-angle_array[array_length-1])/(timer_array[0]-timer_array[array_length-1]);

			//parametros
			lin_kp=0.0105;
			lin_ki=0;
			lin_kd=0.0004;
			rot_kp=6.19;
			rot_ki=754;
			rot_kd=0.0295;
			lin_kd_avg=150;


		//Controlador

			if (lin_position>limit_lin_min &&lin_position<limit_lin_max){       //funciona enquanto dentro das posições permitidas
						if (angle<limit_angle &&angle>(limit_angle*-1)){    	//funciona enquanto dentro do ângulo permitido 
										//Controle PID automatico
										  if (lin_position<(lin_Setpoint+100)&&lin_position>(lin_Setpoint-100)){
											lin_kp=lin_kp*1;
											lin_ki=lin_ki*1;
											lin_kd=lin_kd*1;
											rot_kp=rot_kp*1;
											rot_ki=rot_ki*1;
											rot_kd=rot_kd*1;
										  }
										//Controle PID linear
										  lin_Controller.SetTunings(lin_kp,lin_ki,lin_kd);
										  lin_Controller.Compute();
										  rot_Setpoint=rot_Setpoint+rot_offset;
										//Controle PID rotacional
										  Err_angle=rot_Setpoint-angle;
										  rot_Controller.SetTunings(rot_kp,rot_ki,rot_kd);
										  rot_Controller.Compute();
											
										  if(Output>0){
											Output=map(abs(Output),0,255,setup_zero_speed,255);
											go_right(Output);
											}
											
										  if(Output<0){
											Output=map(abs(Output),0,255,setup_zero_speed,255);
											go_left(Output);
											Output=-Output;
											}
									  
										  if(Output==0){
											go_stop;
											Output=0;
											}
										
						}
						//para o controlador se o angulo for superior ao limite definido
						  else{   
							go_stop();    
							Output=0;
						  }
						
					  
			}
			//manda o carrinho para dentro dos limites caso sejam excedidos
			  else{ 
					  //para o carrinho suavemente para que não bata nas extremidades da estrutura  
						if (lin_position>limit_lin_max){go_right(255);delay(10);}
						if (lin_position<limit_lin_min){go_left(255);delay(10);}
						go_stop();         
						delay(200);
					  //comanda o carrinho para dentro dos limites  
						if(linEnc.read()<limit_lin_min){      //caso acima do limite direito
							while(linEnc.read()<limit_lin_min+800){ 
							go_left(180);
							delay(10);
							go_left(setup_drive_speed);
							}
							go_stop();
						}
						
						if(linEnc.read()>limit_lin_max){      //caso acima do limite esquerdi
							while(linEnc.read()>limit_lin_max-800){
							go_right(180);
							delay(10);
							go_right(setup_drive_speed);
							}
							go_stop();
						}
						
				  }
			if (Serial.available() != 0) {
				leitura_serial = Serial.read();
				//L
				if (leitura_serial == 76) {
					Serial.print("L,");
					Serial.println(angle); 
					leitura = 1;        
				} 
			}
			if (leitura_serial == 80) {
				analogWrite(pin_pwm,0);
			  //comando = false;
			}
			if (leitura == 0)
				delayMicroseconds(8350);
			else {
				delayMicroseconds(7482);
				leitura = 0;
			}
}


//DECLARAÇÃO DE PROCEDIMENTOS
	//mover o carro para a esquerda com velocidade
    void go_left(int velocity){       
       digitalWrite(pin_right,LOW); 
       digitalWrite(pin_left,HIGH);
       analogWrite(pin_pwm,velocity);
      }
      
	//mover o carro para a direita com velocidade
    void go_right(int velocity){      
       digitalWrite(pin_right,HIGH); 
       digitalWrite(pin_left,LOW);
       analogWrite(pin_pwm,velocity);
    }
    
	//para a movimentação
    void go_stop(){                   
      analogWrite(pin_pwm,0);
    }
