function [Obj, Q] = setupQuadObjective(len, N, NumTar, names, action)

Q = zeros(len,len);
Obj=zeros(1,len);
weigths = [N:-1:1];

if strcmp(action,'product')
    
    for k=1:NumTar
        for i=1:N
            index1 = find(strcmp(names, strcat(num2str(k),'b',num2str(i))));
            index2 = find(strcmp(names, strcat('alt_ha',num2str(i))));
            index3 = find(strcmp(names, strcat('z',num2str(i))));
            
            %             Q(index1,index2) = -0.001;
            for p=k:NumTar
                if p~=k
                    index3 = find(strcmp(names, strcat(num2str(p),'b',num2str(i))));
                    Q(index1,index3)=-10*weigths(i);
                end
            end
            %             Q(index2, index3) = -0.00000003;
        end
    end
    
elseif strcmp(action,'sum')
    
    for k=1:NumTar
        for i=1:N
            index1 = strcmp(names, strcat(num2str(k),'b',num2str(i)));
            index2 = strcmp(names, strcat('Pdet',num2str(i)));
            
            Q(index1,index2) = -1*weigths(i);
        end
    end
    
elseif strcmp(action,'cov')
    
    for k=1:NumTar
        for i=1:N
            index1 = find(strcmp(names, strcat(num2str(k),'pt',num2str(i),'_',num2str(i))));
            Obj(1,index1)=2;
        end
    end
    
else
    %     fprintf('There is no objective option for "%s"', action);
    error('There is no objective with given option')
end
%
% for i=1:N
%     index1 = find(strcmp(names, strcat('fx',num2str(i))));%force at x axis
%     index2 = find(strcmp(names, strcat('fy',num2str(i))));%force at y axis
%     index3 = find(strcmp(names, strcat('fz',num2str(i))));%force at z axis
%     
%     Q(index1,index1) = 0.05;
%     Q(index2,index2) = 0.05;
%     Q(index3,index3) = 0.051;
% end

end