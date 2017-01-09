clear
 
 % laplacian matrix used to calculate the connected edges a spanning tree
 % (pws vgainei)
L=-[	-2	1	0	1	0
		1	-2	0	1	0
		0	0	-2	1	1
		1	1	1	-4	1
		0	0	1	1	-2 
	];



x=[ 2 3 4 5 6]'; % mc=4  (noumera ka8e komvou sthn arxi )
y=x;
 
% system model matrices (h times tou systimatos)
A=[ -0.8 .5; .1 -.6];
C=[1 0]; 

% system intinial state +
zx=[4; 4];
zy=0 ;

%estimations (ta arxika estimation prin arxisei to simulation)
zestx=[0; 0]; % a priori
zesty=0 ;	% a priori
zestxm=0 ; % a posteriori
zestym=0 ; % a posteriori

% aggregation vectors of estimations (edw?)

zzy=[  0];
zzesty= [  0];	% a priori
zzestym= [  0]; % a posteriori

% Kalman gain (giati dilwmeno e3 arxhs kai oxi dunamiko?)
	K=[0.9 ; 0.9];
	

for ii=1:15

	zestx= A*zestx; 
	zesty=C*zestx;% a priori
	
	zx=A*zx+.05*rand(1,1);
	zy=C*zx;
	
	% consensus
	
		%collapse measurements
		 x=[zy+1 zy-1 zy-2 zy-1 zy+3 ]';
		
		for jj=1:50
			x=(eye(5)-L/2.6)*x;
			y= [ y x];
		end
		zy=x(1,1);
	
	% end consensus

	
	 zestxm=  zestx+K*(zy-zesty)  ;		% a posteriori
	 zestym=C*zestxm;	% a posteriori
	 zestx=zestxm;
	 zzesty=[zzesty  zesty];
	 zzestym=[zzestym  C*zestxm];
     zzy= [ zzy   zy];
	 


%	add delays to plot consensus procedure  (1/2)
%	 zzesty=[zzesty  zesty*ones(1,50)];
%	  zzestym=[zzestym zestym*ones(1,50)	];
%	  zzy= [ zzy	zy*ones(1,50)	]; 
	 
	 
	 
end

% end of simulation

% plotting


%	add delays to plot consensus procedure  (2/2)
  % plot (y(1,:), 'red')
  %  hold on
 %   plot (y(2,:),'green')
 %  plot (y(3,:),'blue')
 %   plot (y(4,:), 'black')
 %     plot (y(5,:),'magenta')
 

%bita figure
% real, apriori, aposteriori
 plot (zzy(1,:), 'red');
 hold on;
 plot (zzesty(1,:), 'green');
plot (zzestym(1,:), 'blue');
