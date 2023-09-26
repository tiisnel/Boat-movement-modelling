#include <stdio.h>
#include <stdlib.h>
#include "bmdl.cpp"
#include <math.h>

int main(){
	
	float viga=2 /*et kohe alguses "kohal" ei oleks */,suund;
	float ekse;
	float a=500,b=500; // paadile avaldatqav jõud
	bmodel paadimudel;
	
	#define PI 3.14159265
	int i=0; // aeg
	state_vector olek, algolek;
	float xok=-100,yok=-230; // soovitud sihtpunkt. Selle muutmisega saab testida programmi tööd
	
	// paadi alg-asukoht
	algolek.x=10;
	algolek.y=23;
	algolek.t=160;
	algolek.vx=30;
	algolek.vy=20;
	algolek.w=0;
	
	if (algolek.x==xok && algolek.y==yok){
		a=0;
		b=0;
	}
	
	paadimudel.init(algolek,0.1);
	printf("    x\t\t    y\t\t    t\t\t   viga\t\t  vaja(t)\t   A\t\t  B\n");
    while (true){
	    printf("%f", olek.w);
	    if(viga < 1  && fabs(olek.vx)<0.00001 && fabs(olek.vy)<0.00001 && fabs(olek.w)<0.00001){ // kui paat on kohal ja sisuliselt ei liigu enam.
	    	printf("\n\n\nPaat on kohale joudnud. Juhtimismudeli too lopetatud.");
	    	printf("\nKohale joudmiseks kulus %d systeemi ajayhikut.\n\n",i );
			system("pause");
	    	exit(0);
		}
	
	    olek = paadimudel.next_position(a,b);
        i+=1; // j2rgmine ajahetk
	    
	    if(olek.t<0) olek.t=olek.t+360;
	    if(olek.t>360) olek.t=olek.t-360;
	    
	    viga=sqrt(pow(olek.x-xok,2)+pow(olek.y-yok,2));//kaugus soovitud punktist
	    suund=atan2(yok-olek.y,xok-olek.x)*180/PI;
	    
	    
	    suund=olek.t-suund;// suuna viga (kuhu oleks vaja pöörata)
	    if(suund>360)suund-=360;
	    if(suund<0)suund+=360;
	    if (viga<1){
	    	a=0;
	    	b=0;
		}
	    else if(suund<1 || suund >359){
		    a=500;
			b=500;
		}
	    else if(suund>180){
	    	a=0;
	    	b=500;
		}
	    else{
	    	a=500;
	    	b=0;
		}
	    printf("\n");
	    
	    printf("%lf\t%lf\t%lf\t%f\t%f\t%f\t%f\n",olek.x,olek.y,olek.t,viga,suund,a,b);
	}
}
//PROGRAMMI LOPP--------------------------------------------------------------------------------------------------------------------------
	    // esialgne töötav versioon juhtimissüsteemist, praeguseks asendatud ülaloleva versiooniga....
	   // printf("%f\n",suund);
	   /*
	    suund=asin((yok-olek.y)/viga);
	    suund=suund/M_PI*180;  //   90
	                           // 0    0
	                           //  -90
	                           
	    if(olek.x<=xok){
	    	if(olek.y>yok)suund+=360; //4.veerand
	    	//else 1.veerand, muutust pole vaja
		}
		if(olek.x>xok){
			suund=90+(90-suund); //2. ja 3. veerand
		}
		                        //      90
		                        // 180      0(360)
		                        //     270
		if(viga==0)suund=0;
		
       ekse=500;// liiga suur nr :)

		if(suund-olek.t<ekse){
			if(suund-olek.t>=0){
				ekse=suund-olek.t;
				a=0;
	        	b=500;
			}
		}
		
		if(suund-olek.t-360<ekse){
			if(suund-olek.t-360>=0){
			    ekse=fabs(suund-olek.t-360);
				a=0;
				b=500;	
			}
		}
		if(olek.t-suund<ekse){
			if(olek.t-suund>=0){
		        ekse=olek.t-suund;
		        a=500;
		        b=0;
		    }
	    }
	    if(olek.t+360-suund<ekse){
	    	if(olek.t+360-suund>=0){
	        	ekse=olek.t+360-suund;
		        a=500;
		        b=0;
		    }
		}
	    if(ekse<1){
	    	a=500;
	    	b=500;
		}
		if(viga<1){
			a=0;
			b=0;
		}
	    */
	    

