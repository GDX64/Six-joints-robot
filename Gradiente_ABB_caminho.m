close all
clc

ft=100; %tamanho dos eixos
mi=4;
x0=1;
y0=1;
width=1000;
height=1000;
sc=1; %scale factor for drawing
set(gcf,'units','points','position',[x0,y0,width,height]) %isso � pra aumentar a janela

%% Caminhos
%cirnferencia

x = 200 + 400*cosd(0:2:360);
z = 800+300*sind(0:2:360);
y = 500*ones(1,length(x));

%cora��o
%  t = -3:.1:3;
%  x = 5+2*(1.6*(power(sin(t),3)));
%  z = 10+2*(1.3*cos(t)-0.5*cos(2*t)-0.2*cos(3*t)-0.1*cos(4*t));
%  y = 10+zeros(1,length(x));

%% Objetivos e orienta��o

%coloque aqui o objetivo
obj1=[x; y; z]*sc; %x, y, e z tem que ser as linhas
obvec={obj1}; %vetor de objetivos
orientacao=[1 0 0 0 0 -1 0 1 0];

%links
L1=3.5*100;
L2=10.5*100;
L3=10*100;
L4=0;
L5=2.1*100;

 %% Jacobiano
if ~exist('Jf')
    syms th1 th2 th3 th4 th5 th6
    
    %%modelo do rob�

    F0=rot('x',0);
    f1=F0*rot('z',th1)*trans([L1 0 0])*rot('x',-pi/2);
    f2=f1*rot('z',th2-pi/2)*trans([L2 0 0]);
    f3=f2*rot('z',th3)*rot('x',-pi/2);
    f4=f3*rot('z',th4)*trans([0 0 L3+L4])*rot('x',pi/2);
    f5=f4*rot('z',th5)*rot('x',-pi/2);
    f6=f5*rot('z',th6)*trans([0 0 L5]);

    %%linearizando o rob�
    Jf=jacobian([f6(1:3,1);f6(1:3,2);f6(1:3,3);f6(1:3,4)]);
    Jf=matlabFunction(Jf);
    f1=matlabFunction(f1);
    f2=matlabFunction(f2);
    f3=matlabFunction(f3);
    f4=matlabFunction(f4);
    f5=matlabFunction(f5);
    f6=matlabFunction(f6);
end

%escolhendo valores iniciais de juntas
th1=0;
th2=0;
th3=0;
th4=0;
th5=0;
th6=0;

%atualizando frames
F1=f1(th1);
F2=f2(th1,th2);
F3=f3(th1,th2,th3);
F4=f4(th1,th2,th3,th4);
F5=f5(th1,th2,th3,th4,th5);
F6=f6(th1,th2,th3,th4,th5,th6);

for v=1:length(obvec)
    obj=obvec{v};

    for n=1:length(obj)

        u=0.5; %passo de converg�ncia
        E=reshape(F6(1:3,:),[1 12])-[orientacao, obj(:,n)']; %Erro
        n1=0; %numero de itera��es
        delta=1E-5; %anti singularidade

        while E*E'>0.001;
            n1=n1+1;
            E=reshape(F6(1:3,:),[1 12])-[orientacao, obj(:,n)']; %Erro
            DX=E'; %varia��o de X
            fe=Jf(th1,th2,th3,th4,th5,th6); %fun��o objetivo linearizada
            if det(fe'*fe)==0 %corrigindo singularidade
                fe=pinv(fe'*fe+delta*eye(6,6))*fe';
            else
                fe=pinv(fe'*fe)*fe';
            end
            DQ=u*fe*DX; %encontrando Delta Q
            %atualizando juntas
            th1=th1-DQ(1,1);
            th2=th2-DQ(2,1);
            th3=th3-DQ(3,1);
            th4=th4-DQ(4,1);
            th5=th5-DQ(5,1);
            th6=th6-DQ(6,1);
            %colocando os valores de juntas no rob�

            F6=f6(th1,th2,th3,th4,th5,th6);

            if n1>100 %caso n�o convirja, sorteamos novos valores
                th4=rand(1,1)*pi;
                th5=rand(1,1)*pi;
                th6=rand(1,1)*pi;
                n1=0;
            end
        end

        %calculando frames pra plotar
        F1=f1(th1);
        F2=f2(th1,th2);
        F3=f3(th1,th2,th3);
        F4=f4(th1,th2,th3,th4);
        F5=f5(th1,th2,th3,th4,th5);

        nc(n)=n1;
        
        plot3(obj(1,1:n),obj(2,1:n),obj(3,1:n), 'k', 'lineWidth', 5)
        hold on
        %plotando caminho
        if v>1
            for i=1:v-1
                plot3(obvec{i}(1,:),obvec{i}(2,:),obvec{i}(3,:), 'k', 'lineWidth', 5)
            end
        end
        frames={F0 F1 F2 F3 F4 F5 F6};

        for i=1:length(frames)
            %link
            if i<length(frames)
               plot3([frames{i}(1,4) frames{i+1}(1,4)],[frames{i}(2,4) frames{i+1}(2,4)],[frames{i}(3,4) frames{i+1}(3,4)], 'y', 'linewidth', 6) 
            end
            F1=frames{i};

            plot3(F1(1,4) , F1(2,4) , F1(3,4) , 'om', 'linewidth', 2 , 'markersize', 15);
            text(F1(1,4) , F1(2,4) , F1(3,4)-0.2 , strcat('\{', mat2str(i-1), '\}'))
            % Eixo-x no rob� no frame
            if (i==length(frames) | i==1)
                F1(1:3,1:3)=F1(1:3,1:3)*ft;
                plot3([F1(1,4) F1(1,4)+F1(1,1)] , [F1(2,4) F1(2,4)+F1(2,1)] , [F1(3,4) F1(3,4)+F1(3,1)] , 'b', 'linewidth', 2)
                text(F1(1,4)+F1(1,1) , F1(2,4)+F1(2,1) , F1(3,4)+F1(3,1) , strcat('x_{\{', mat2str(i-1), '\}}'))
                % Eixo-y no rob� no frame
                plot3([F1(1,4) F1(1,4)+F1(1,2)] , [F1(2,4) F1(2,4)+F1(2,2)] , [F1(3,4) F1(3,4)+F1(3,2)] , 'r', 'linewidth', 2)
                text(F1(1,4)+F1(1,2) , F1(2,4)+F1(2,2) , F1(3,4)+F1(3,2) , strcat('y_{\{', mat2str(i-1), '\}}'))
                % Eixo-z no rob� no frame
                plot3([F1(1,4) F1(1,4)+F1(1,3)] , [F1(2,4) F1(2,4)+F1(2,3)] , [F1(3,4) F1(3,4)+F1(3,3)] , 'g', 'linewidth', 2)
                text(F1(1,4)+F1(1,3) , F1(2,4)+F1(2,3) , F1(3,4)+F1(3,3) , strcat('z_{\{', mat2str(i-1), '\}}'))
                axis(1*[-1000 1000 -1000 1000 0 1500])
            end
            view(130,30) %ajusta a vis�o do plot

        end

        grid on
        hold off
        drawnow


    end
end

