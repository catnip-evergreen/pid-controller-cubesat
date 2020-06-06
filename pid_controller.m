%% loop to test and find the optimum proportional, integral and derivative gain using the existing simulink model and the tested code for the whole model.
%PiD CONTROLLER WITH variable kp AND KD, and with the torque and the momentum restrictions 
clc;
clear all;
%% initialisations
T=2000;
dt=1;% reduce it after the whole program starts running
t=0:dt:T;
dp=1e-6;% the variation of the constant Kp
inertia=[0.006469878086433,0,0;0,0.021899163044106,0;0,0,0.022270958869461];
wi=[-0.0019976663;-0.0000204341;0.0003577060];% initial angular velocity obtained after detumbling.
Kp=0;
q_desired=[0;0;0;1];
E_q=[0,0,0,0;
     0,0,0,0;
     0,0,0,0;
     0,0,0,0];
 store_kp=zeros(2001,1);
 store_kd=zeros(101,1);
 store_ki=zeros(101,1);
 store_v_count=zeros(2001,101,101);
%% main code
count=0;
for Kp=0.001:dp:0.003

    Kd=0;
   count=count+1;
   counter=0;
   store_kp(count,1)=Kp;
for Kd=0:dp:0.0001

   counter=counter+1;
         store_kd(counter,1)=Kd;
counteri=0;
Ki=0;
for Ki=0:dp:0.0001
    counteri=counteri+1;
    store_ki(counteri,1)=Ki;    
  v_count=0;
  alpha_old=[0;0;0];
  q_old=[0;1;0;0];
  q_new=[0;1;0;0];
  alpha_new=[0;0;0];
  omega_old=wi;
  e=[0;0;0;0];
  torque=[0;0;0];
  omega_new=wi;
  theta=[0;0;0];
for ik =1:length(t)
        rx=[1,0,0;0,cos(theta(1)),-sin(theta(1));0,sin(theta(1)),cos(theta(1))];
        ry=[cos(theta(2)),0,sin(theta(2));0,1,0;-sin(theta(2)),0,cos(theta(2))];
        rz=[cos(theta(3)),-sin(theta(3)),0;sin(theta(3)),cos(theta(3)),0;0,0,1];
        b=rx*ry*rz; 
        E_q=[q_old(4),-q_old(3),q_old(2);
             q_old(3),q_old(4),-q_old(1);
             -q_old(2),q_old(1),q_old(4);
             -q_old(1),-q_old(2),-q_old(3)];
        q_dot=(0.5*E_q*omega_new);
        q_new=q_new+q_dot;
        q_new = q_new/norm(q_new);
        e=(q_desired-q_new);   
        omega_new=omega_old+(dt*alpha_old);
        conv_ang_vel=b*omega_new;
        alpha_new(1)=(1/inertia(1,1))*(((inertia(2,2)-inertia(3,3))*conv_ang_vel(2)*conv_ang_vel(3))+torque(1));
        alpha_new(2)=(1/inertia(2,2))*(((inertia(3,3)-inertia(1,1))*conv_ang_vel(1)*conv_ang_vel(3))+torque(2));
        alpha_new(3)=(1/inertia(3,3))*(((inertia(1,1)-inertia(2,2))*conv_ang_vel(2)*conv_ang_vel(1))+torque(3));
        alpha_=b*alpha_new;
        torque=-(Kd*[e(1);e(2);e(3)])-(Kp*conv_ang_vel)-(Ki*alpha_);
        
        
        q_old=q_new;   
        theta=(omega_old*dt)+(0.5*alpha_old*dt*dt);
        alpha_old=b*alpha_new;
        omega_old=b*omega_new;
        v_count=v_count+1; 
        store_v_count(count,counter,counteri)=(v_count*dt);
        if((norm(torque)>=(10^(-4)))||(norm(inertia*omega_new)>=(1.5*10^(-3))))
            store_v_count(count,counter,counteri)=10000;
        break;
        end      
        if(abs(omega_new(1))<(10^(-6)) && abs(omega_new(2))<(10^(-6)) && abs(omega_new(3))<(10^(-6)) )
        disp('yayya');
            break;
        end  
end
end
end
end