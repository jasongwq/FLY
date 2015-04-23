#include <reg51.h>
char led_mod[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
char key_buf[] = {0xee,0xde,0xbe,0x7e,0xed,0xdd,0xbd,0x7d,0xeb,0xdb,0xbb};
void delay(unsigned int time);
char getKey(void) {
char key_scan[] = {0xef,0xdf,0xbf,0x7f};
char i=0,j=0;
for (i=0;i<4;i++) {
       P3= key_scan[i];
       if(P3!= 0xff){
          for(j=0;j<16;j++){
             if(key_buf[j]== P3)return j;
}
}
}
       return - 1;
}           


void main(void){
    char key0=0;
    char key2=0;
	  char key3=0;
	  char   s=0;
    P0=0x00;
    P2=0x00;
first:
	   P0=0x00;
     P2=0x00;
     while( 1){
		 key0=getKey();
     if(key0!= - 1)
     P0 = led_mod[key0];
	    delay(200);
		 if(key0!=-1)
      goto next ;}
next: delay(200);
			while(1){
			key2=getKey();
			if(key2!=-1)
		  P2= led_mod[key2];
			delay(200);
		  if(key2!=-1)
			goto fin;} 		
fin:  delay(500);
		  s=key0+key2;
		 while(key3!=-1){
      key3=getKey();
       if(key3==0xbb){
       P0=led_mod[s/10];
       P2=led_mod[s%10];
        delay(200);}
       if(key3!=0xbb){
        P0=led_mod[0x7f];
        P2=led_mod[0x7f];
				 delay(200);
}}}


void delay(unsigned int time){
unsigned int k=0;
for(;time>0;time--)
for(k=0;k<125;k++);}
