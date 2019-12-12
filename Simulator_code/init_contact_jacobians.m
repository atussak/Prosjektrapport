
% --------------------------------------------------------
% * Find symbolic expressions for the possible 
%   contact Jacobians and their derivatives.
% * We can have n possible contact Jacobians (one per link).
% * The distance from the joint to the contact point is to be
%   found in the last elements of q.
% --------------------------------------------------------


global q qd Q Qdiff n N l Jc_func Jcd_func D_func Rz_func


% Contact Jacobians
all_Jc  = sym('Jc%d', [2 N n]);
Jc = sym('Jc%d', [2 N]);

% Derivative of contact Jacobians
all_Jcd  = sym('Jc%d', [2 N n]);
Jcd = sym('Jc%d', [2 N]);


for link = 1:n % For every link an obstacle can be in contact with
    T = D_func(q(n+1),q(n+2));
    for curr_link = 1:link-1
        T = T*Rz_func(q(curr_link))*D_func(l,0);
    end
    T = T*Rz_func(q(link))*D_func(q(n+2+link));
    x = T(1,4);
    y = T(2,4);
    for i = 1:N
        Jc(1,i) = diff(x, q(i));
        Jc(2,i) = diff(y, q(i));
    end
    Jc = simplify(Jc);
    all_Jc(:,:,link)  = Jc;
end      
  %% Derivative of Jacobian
  % Make variables time dependent to diff wrt time
  for row = 1:2
      for col = 1:N
          for i = 1:N
            Jc(row,col) = subs(Jc(row,col), q(i), Q(i));
          end
      end
  end
  
  for row = 1:2
      for col = 1:N
          Jcd(row,col) = diff(Jc(row,col), t);
      end
  end
  
  for row = 1:2
      for col = 1:N
          for i = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Qdiff(i), qd(i));
          end
          for i = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Q(i), q(i));
            Jcd(row, col) = subs(Jcd(row, col), Qd(i), qd(i));
          end
      end
  end
  
  all_Jcd(:,:,link) = Jcd;
  
end

%% Make functions for faster calculation at insertion of numbers
Jc_func  = matlabFunction(all_Jc, 'vars', {q});
Jcd_func = matlabFunction(all_Jcd, 'vars', {q, qd});

