%Funcio per recalcular correspondencies, coneixen una estimacio inicial de
%la rotacio

function [P1,P2,error]=acc_registration(P1i,P2i,R,t)
%R i t matrius de transformacio de P1 a P2.
% P1i i P2i  3xn : punts 3d dels dos nuvols
% P1 i P2 3xn  : correspondencies segons ICP (ClosestPoint)
n1=size(P1i,1);
n2=size(P2i,1);
%tri = delaunayn(P2i')';
for i=1:n1
    P1it(i,:)=P1i(i,:)*R+t;
    %[k,d] = dsearchn(P2i',tri,P1it(:,i)');
    [k,d] = dsearchn(P2i,P1it(i,:));
    P1(i,:)=P1i(i,:);
    P2(i,:)=P2i(k,:);
    error(i)=d;
end

error = mean(error);

