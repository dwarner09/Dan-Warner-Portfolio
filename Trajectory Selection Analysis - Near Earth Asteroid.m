%% Project - Flight times and Delta V's to Asteroid 2001 F85

% Start 2038-01-01
% End 2042-01-01

% This code calculates dV as a function of Departure and Arrival Date

%% Constants
mu_sun = 1.32712428E11;
mu_earth = 3.986004415E5;


r_earth = 6378.1363;
re = r_earth + 300;
ve = sqrt(mu_earth/re);

load fouryearstatevectors

%% Determine Vectors

vi = zeros(366,366);
vf = zeros(366,366);
DV1_mag_L = NaN(366,366);
DV2_mag_L = NaN(366,366);
DV_Dep_L = NaN(366,366);
DV1_mag_S = NaN(366,366);
DV2_mag_S = NaN(366,366);
DV_Dep_S = NaN(366,366);

DV3_mag_L = NaN(366,366);
DV4_mag_L = NaN(366,366);
DV_Ret_L = NaN(366,366);
DV3_mag_S = NaN(366,366);
DV4_mag_S = NaN(366,366);
DV_Ret_S = NaN(366,366);
TOF_array = zeros(366,366);

a = zeros(366,366);
%% Transfer - TrA > 180*
for k = 1:268  % k is earth departure date, each step is 30 days
    for j = k+12:k+98 % j is TOF, each step is 30 days
        TOF = time4(j,1) - time4(k,1);
        
        %% Vectors
        r1 = [e_pos4(k,1), e_pos4(k,2), e_pos4(k,3)];
        v1 = [e_vel4(k,1), e_vel4(k,2), e_vel4(k,3)];
        r2 = [a_pos4(j,1), a_pos4(j,2), a_pos4(j,3)];
        v2 = [a_vel4(j,1), a_vel4(j,2), a_vel4(j,3)];

        %% Step 1 Transfer Angle >180

        TrA = acos(dot(r2, r1)/(norm(r1)*norm(r2)));
        TrA = 2*pi-TrA;

        %% Step 2 Geometric Quantities

        c = sqrt(norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cos(TrA)); %km
        s = 0.5*(norm(r1) + norm(r2) +c); %km

        %% Step 3 Determine if uses elliptical or hyperbolic

            % for TrA > 180 
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)); %s

            % for TrA < 180
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)); %s


            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)))/(60*60*24);

        %% Step 4 Short or Long

        am = s/2;
        alpham = pi();
        nm = sqrt(mu_sun/am^3);
        betam0 = 2*asin(sqrt((s-c)/s));
        
        if (TrA>=0) && (TrA<=pi) %transfer angle < 180, B = Bo
            betam = betam0;
            B_t = 1;
        else % transfer angle > 180 B = -Bo
            betam = -betam0;
            B_t = 0;
        end

        TOF_min = (1/nm*((alpham - betam)-(sin(alpham)-sin(betam))))/(60*60*24);
        
        if  TOF < TOF_min % short
            A_t = 1;
        else % long
            A_t = 0;
        end
      
        %% Step 5/6  Iteratively solve TOF to find alpha
       
        % initial guess
        ai = [am];
        
        if TOF < TOF_p %hyperbolic transfer
            if B_t == 1 % TrA < 180
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))-(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            else
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))+(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            end
            
        else      %% eliptical transfer
            if (B_t == 1) && (A_t ==1) % quadrant 1 alpha = alphao, beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==0) && (A_t ==1) %% quadrant 3 alpha = alphao beta = -betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==1) && (A_t ==0) %% quadrant 2 alpha = 2pi - alphao beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi-2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            else %% quadrant 4 alpha = 2pi - alphao beta = -beta0
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi - 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            end
        end
        
        if exitflag > 0
            a = a;
        else
            a=NaN;
        end

        %% Step 7

        p = 4*a*(s - norm(r1))*(s - norm(r2))/c^2*(sin((alpha+beta)/2)^2);

        e = sqrt(1-p/a);

        SE = -mu_sun/(2*a);

        %% Step 8
        TA1 = acos(1/e*(p/norm(r1)-1));
        TA2 = acos(1/e*(p/norm(r2)-1));

        if TrA == TA2 - TA1
            TA2 = TA2;
            TA1 = TA1;
        elseif TrA == -TA2 - TA1
            TA2 = -TA2;
            TA1 = TA1;
        elseif TrA == -TA2 + TA1
            TA2 = -TA2;
            TA1 = -TA1;
        else 
            TA2 = TA2;
            TA1 = -TA1;
        end

        %% F G
        D_TA2 = TA2 - TA1;
        D_TA1 = TA1 - TA2;

        f = 1 - (norm(r2)/p*(1-cos(D_TA2)));
        g = norm(r2)*norm(r1)/(sqrt(mu_sun*p))*sin(D_TA2);
        f_prime = sqrt(mu_sun/p)*tan(D_TA2/2)*((1-cos(D_TA2))/p - 1/norm(r2) - 1/norm(r1));
        g_prime = 1 - (norm(r1)/p)*(1-cos(D_TA2));

        vi = (r2-f*r1)/g;
        vf = f_prime*r1+g_prime*vi;

        %% Delta V

        DV1 = vi - v1;
        DV1_mag_L(j,k) = norm(DV1);
        DV2 = v2 - vf;
        DV2_mag_L(j,k) = norm(DV2);
        DV_Dep_L(j,k) = DV1_mag_L(j,k) + DV2_mag_L(j,k);
        
        %% Patched Conics

        vinf_out = DV_Dep_L(j,k);
        mag_vinf_out = norm(vinf_out);

        SE_ehyp = norm(vinf_out)^2/2;

        vp1 = sqrt(2*(SE_ehyp + mu_earth/re));
        DVd = vp1 - ve;

        DV_Dep_L(j,k) = DVd;
    end
end



%% Transfer for TrA < 180
for k = 1:306  % k is earth departure date, each step is 30 days
    for j = k+10:k+60 % j is TOF, each step is 30 days
        TOF = time4(j,1) - time4(k,1);
        
        %% Vectors
        r1 = [e_pos4(k,1), e_pos4(k,2), e_pos4(k,3)];
        v1 = [e_vel4(k,1), e_vel4(k,2), e_vel4(k,3)];
        r2 = [a_pos4(j,1), a_pos4(j,2), a_pos4(j,3)];
        v2 = [a_vel4(j,1), a_vel4(j,2), a_vel4(j,3)];

        %% Step 1 Transfer Angle

        TrA = acos(dot(r2, r1)/(norm(r1)*norm(r2)));

        %% Step 2 Geometric Quantities

        c = sqrt(norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cos(TrA)); %km
        s = 0.5*(norm(r1) + norm(r2) +c); %km

        %% Step 3 Determine if uses elliptical or hyperbolic

            % for TrA > 180 
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)); %s

            % for TrA < 180
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)); %s

        if (TrA>=0) && (TrA<=pi)
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)))/(60*60*24);
        else
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)))/(60*60*24);
        end


        %% Step 4 Short or Long

        am = s/2;
        alpham = pi();
        nm = sqrt(mu_sun/am^3);
        betam0 = 2*asin(sqrt((s-c)/s));
        
        if (TrA>=0) && (TrA<=pi) %transfer angle < 180, B = Bo
            betam = betam0;
            B_t = 1;
        else % transfer angle > 180 B = -Bo
            betam = -betam0;
            B_t = 0;
        end

        TOF_min = (1/nm*((alpham - betam)-(sin(alpham)-sin(betam))))/(60*60*24);
        
        if  TOF < TOF_min % short
            A_t = 1;
        else % long
            A_t = 0;
        end
      
        %% Step 5/6  Iteratively solve TOF to find alpha
       
        % initial guess
        ai = [am];
        
        if TOF < TOF_p %hyperbolic transfer
            if B_t == 1 % TrA < 180
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))-(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            else
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))+(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            end
            
        else      %% eliptical transfer
            if (B_t == 1) && (A_t ==1) % quadrant 1 alpha = alphao, beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==0) && (A_t ==1) %% quadrant 3 alpha = alphao beta = -betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==1) && (A_t ==0) %% quadrant 2 alpha = 2pi - alphao beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi-2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            else %% quadrant 4 alpha = 2pi - alphao beta = -beta0
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi - 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            end
        end
        
        if exitflag > 0
            a = a;
        else
            a=0;
        end

        %% Step 7

        p = 4*a*(s - norm(r1))*(s - norm(r2))/c^2*(sin((alpha+beta)/2)^2);

        e = sqrt(1-p/a);

        SE = -mu_sun/(2*a);

        %% Step 8
        TA1 = acos(1/e*(p/norm(r1)-1));
        TA2 = acos(1/e*(p/norm(r2)-1));

        if TrA == TA2 - TA1
            TA2 = TA2;
            TA1 = TA1;
        elseif TrA == -TA2 - TA1
            TA2 = -TA2;
            TA1 = TA1;
        elseif TrA == -TA2 + TA1
            TA2 = -TA2;
            TA1 = -TA1;
        else 
            TA2 = TA2;
            TA1 = -TA1;
        end

        %% F G
        D_TA2 = TA2 - TA1;
        D_TA1 = TA1 - TA2;

        f = 1 - (norm(r2)/p*(1-cos(D_TA2)));
        g = norm(r2)*norm(r1)/(sqrt(mu_sun*p))*sin(D_TA2);
        f_prime = sqrt(mu_sun/p)*tan(D_TA2/2)*((1-cos(D_TA2))/p - 1/norm(r2) - 1/norm(r1));
        g_prime = 1 - (norm(r1)/p)*(1-cos(D_TA2));

        vi = (r2-f*r1)/g;
        vf = f_prime*r1+g_prime*vi;

        %% Delta V

        DV1 = vi - v1;
        DV1_mag_S(j,k) = norm(DV1);
        DV2 = v2 - vf;
        DV2_mag_S(j,k) = norm(DV2);
        DV_Dep_S(j,k) = DV1_mag_S(j,k) + DV2_mag_S(j,k);
        
                %% Patched Conics

        vinf_out = DV_Dep_L(j,k);
        mag_vinf_out = norm(vinf_out);

        SE_ehyp = norm(vinf_out)^2/2;

        vp1 = sqrt(2*(SE_ehyp + mu_earth/re));
        DVd = vp1 - ve;

        DV_Dep_L(j,k) = DVd;
    end
end

%% Return Trajectory

for k = 1:268  % k is earth departure date, each step is 30 days
    for j = k+12:k+98 % j is TOF, each step is 30 days
        TOF = time4(j,1) - time4(k,1);
        
        %% Vectors
        r1 = [a_pos4(k,1), a_pos4(k,2), a_pos4(k,3)];
        v1 = [a_vel4(k,1), a_vel4(k,2), a_vel4(k,3)];
        r2 = [e_pos4(j,1), e_pos4(j,2), e_pos4(j,3)];
        v2 = [e_vel4(j,1), e_vel4(j,2), e_vel4(j,3)];

        %% Step 1 Transfer Angle >180

        TrA = acos(dot(r2, r1)/(norm(r1)*norm(r2)));
        TrA = 2*pi-TrA;

        %% Step 2 Geometric Quantities

        c = sqrt(norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cos(TrA)); %km
        s = 0.5*(norm(r1) + norm(r2) +c); %km

        %% Step 3 Determine if uses elliptical or hyperbolic

            % for TrA > 180 
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)); %s

            % for TrA < 180
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)); %s

        if (TrA>=0) && (TrA<=pi)
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)))/(60*60*24);
        else
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)))/(60*60*24);
        end


        %% Step 4 Short or Long

        am = s/2;
        alpham = pi();
        nm = sqrt(mu_sun/am^3);
        betam0 = 2*asin(sqrt((s-c)/s));
        
        if (TrA>=0) && (TrA<=pi) %transfer angle < 180, B = Bo
            betam = betam0;
            B_t = 1;
        else % transfer angle > 180 B = -Bo
            betam = -betam0;
            B_t = 0;
        end

        TOF_min = (1/nm*((alpham - betam)-(sin(alpham)-sin(betam))))/(60*60*24);
        
        if  TOF < TOF_min % short
            A_t = 1;
        else % long
            A_t = 0;
        end
      
        %% Step 5/6  Iteratively solve TOF to find alpha
       
        % initial guess
        ai = [am];
        
        if TOF < TOF_p %hyperbolic transfer
            if B_t == 1 % TrA < 180
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))-(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            else
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))+(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            end
            
        else      %% eliptical transfer
            if (B_t == 1) && (A_t ==1) % quadrant 1 alpha = alphao, beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==0) && (A_t ==1) %% quadrant 3 alpha = alphao beta = -betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==1) && (A_t ==0) %% quadrant 2 alpha = 2pi - alphao beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi-2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            else %% quadrant 4 alpha = 2pi - alphao beta = -beta0
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi - 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            end
        end
        
        if exitflag > 0
            a = a;
        else
            a=NaN;
        end

        %% Step 7

        p = 4*a*(s - norm(r1))*(s - norm(r2))/c^2*(sin((alpha+beta)/2)^2);

        e = sqrt(1-p/a);

        SE = -mu_sun/(2*a);

        %% Step 8
        TA1 = acos(1/e*(p/norm(r1)-1));
        TA2 = acos(1/e*(p/norm(r2)-1));

        if TrA == TA2 - TA1
            TA2 = TA2;
            TA1 = TA1;
        elseif TrA == -TA2 - TA1
            TA2 = -TA2;
            TA1 = TA1;
        elseif TrA == -TA2 + TA1
            TA2 = -TA2;
            TA1 = -TA1;
        else 
            TA2 = TA2;
            TA1 = -TA1;
        end

        %% F G
        D_TA2 = TA2 - TA1;
        D_TA1 = TA1 - TA2;

        f = 1 - (norm(r2)/p*(1-cos(D_TA2)));
        g = norm(r2)*norm(r1)/(sqrt(mu_sun*p))*sin(D_TA2);
        f_prime = sqrt(mu_sun/p)*tan(D_TA2/2)*((1-cos(D_TA2))/p - 1/norm(r2) - 1/norm(r1));
        g_prime = 1 - (norm(r1)/p)*(1-cos(D_TA2));

        vi = (r2-f*r1)/g;
        vf = f_prime*r1+g_prime*vi;

        %% Delta V

        DV3 = vi - v1;
        DV3_mag_L(j,k) = norm(DV3);
        DV4 = v2 - vf;
        DV4_mag_L(j,k) = norm(DV4);
        DV_Ret_L(j,k) = DV3_mag_L(j,k) + DV4_mag_L(j,k);
    end
end



%% Transfer for TrA < 180
for k = 1:306 % k is earth departure date, each step is 30 days
    for j = k+10:k+60 % j is TOF, each step is 30 days
        TOF = time4(j,1) - time4(k,1);
        
        %% Vectors
        r1 = [a_pos4(k,1), a_pos4(k,2), a_pos4(k,3)];
        v1 = [a_vel4(k,1), a_vel4(k,2), a_vel4(k,3)];
        r2 = [e_pos4(j,1), e_pos4(j,2), e_pos4(j,3)];
        v2 = [e_vel4(j,1), e_vel4(j,2), e_vel4(j,3)];

        %% Step 1 Transfer Angle

        TrA = acos(dot(r2, r1)/(norm(r1)*norm(r2)));
       

        %% Step 2 Geometric Quantities

        c = sqrt(norm(r1)^2 + norm(r2)^2 - 2*norm(r1)*norm(r2)*cos(TrA)); %km
        s = 0.5*(norm(r1) + norm(r2) +c); %km

        %% Step 3 Determine if uses elliptical or hyperbolic

            % for TrA > 180 
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)); %s

            % for TrA < 180
            % TOF_p = 1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)); %s

        if (TrA>=0) && (TrA<=pi)
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) - (s-c)^(3/2)))/(60*60*24);
        else
            TOF_p = (1/3*sqrt(2/mu_sun)*(s^(3/2) + (s-c)^(3/2)))/(60*60*24);
        end


        %% Step 4 Short or Long

        am = s/2;
        alpham = pi();
        nm = sqrt(mu_sun/am^3);
        betam0 = 2*asin(sqrt((s-c)/s));
        
        if (TrA>=0) && (TrA<=pi) %transfer angle < 180, B = Bo
            betam = betam0;
            B_t = 1;
        else % transfer angle > 180 B = -Bo
            betam = -betam0;
            B_t = 0;
        end

        TOF_min = (1/nm*((alpham - betam)-(sin(alpham)-sin(betam))))/(60*60*24);
        
        if  TOF < TOF_min % short
            A_t = 1;
        else % long
            A_t = 0;
        end
      
        %% Step 5/6  Iteratively solve TOF to find alpha
       
        % initial guess
        ai = [am];
        
        if TOF < TOF_p %hyperbolic transfer
            if B_t == 1 % TrA < 180
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))-(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            else
                [a,fval,exitflag,out] = fsolve(@(a)((sqrt(a^3/mu_sun))*(sinh(2*asinh(sqrt(s/(2*abs(a)))))-(2*asinh(sqrt(s/(2*abs(a)))))+(sinh(2*asinh(sqrt((s-c)/(2*abs(a)))))-(2*asinh(sqrt((s-c)/(2*abs(a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asinh(sqrt(s/(2*abs(a))));
                beta = 2*asinh(sqrt((s-c)/(2*abs(a))));
            end
            
        else      %% eliptical transfer
            if (B_t == 1) && (A_t ==1) % quadrant 1 alpha = alphao, beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==0) && (A_t ==1) %% quadrant 3 alpha = alphao beta = -betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*((2*asin(sqrt(s/(2*a)))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            elseif (B_t ==1) && (A_t ==0) %% quadrant 2 alpha = 2pi - alphao beta = betao
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))-2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi-2*asin(sqrt(s/(2*a)));
                beta = 2*asin(sqrt((s-c)/(2*a)));
            else %% quadrant 4 alpha = 2pi - alphao beta = -beta0
                [a,fval,exitflag,out]  = fsolve(@(a)(1/(sqrt(mu_sun/a^3))*(((2*pi-2*asin(sqrt(s/(2*a))))+2*asin(sqrt((s-c)/(2*a)))-(sin(2*pi-2*asin(sqrt(s/(2*a))))-sin(-2*asin(sqrt((s-c)/(2*a)))))))-(TOF*60*60*24)), ai);
                alpha = 2*pi - 2*asin(sqrt(s/(2*a)));
                beta = -2*asin(sqrt((s-c)/(2*a)));
            end
        end
        
        if exitflag > 0
            a = a;
        else
            a=0;
        end

        %% Step 7

        p = 4*a*(s - norm(r1))*(s - norm(r2))/c^2*(sin((alpha+beta)/2)^2);

        e = sqrt(1-p/a);

        SE = -mu_sun/(2*a);

        %% Step 8
        TA1 = acos(1/e*(p/norm(r1)-1));
        TA2 = acos(1/e*(p/norm(r2)-1));

        if TrA == TA2 - TA1
            TA2 = TA2;
            TA1 = TA1;
        elseif TrA == -TA2 - TA1
            TA2 = -TA2;
            TA1 = TA1;
        elseif TrA == -TA2 + TA1
            TA2 = -TA2;
            TA1 = -TA1;
        else 
            TA2 = TA2;
            TA1 = -TA1;
        end

        %% F G
        D_TA2 = TA2 - TA1;
        D_TA1 = TA1 - TA2;

        f = 1 - (norm(r2)/p*(1-cos(D_TA2)));
        g = norm(r2)*norm(r1)/(sqrt(mu_sun*p))*sin(D_TA2);
        f_prime = sqrt(mu_sun/p)*tan(D_TA2/2)*((1-cos(D_TA2))/p - 1/norm(r2) - 1/norm(r1));
        g_prime = 1 - (norm(r1)/p)*(1-cos(D_TA2));

        vi = (r2-f*r1)/g;
        vf = f_prime*r1+g_prime*vi;

        %% Delta V

        DV3 = vi - v1;
        DV3_mag_S(j,k) = norm(DV3);
        DV4 = v2 - vf;
        DV4_mag_S(j,k) = norm(DV4);
        DV_Ret_S(j,k) = DV3_mag_S(j,k) + DV4_mag_S(j,k);
    end
end
%%

save('4yearroundtripresults.mat')