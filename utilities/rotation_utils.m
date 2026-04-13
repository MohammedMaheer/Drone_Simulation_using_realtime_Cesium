function R = rotation_utils(operation, varargin)
% ROTATION_UTILS  Collection of rotation matrix utilities.
%
%   R = rotation_utils('euler2dcm', phi, theta, psi)
%   R = rotation_utils('dcm2euler', R)
%   R = rotation_utils('rotx', angle)
%   R = rotation_utils('roty', angle)
%   R = rotation_utils('rotz', angle)
%   R = rotation_utils('angle_axis', axis, angle)

    switch lower(operation)
        case 'euler2dcm'
            phi = varargin{1}; theta = varargin{2}; psi = varargin{3};
            R = rotz_mat(psi) * roty_mat(theta) * rotx_mat(phi);

        case 'dcm2euler'
            Rm = varargin{1};
            theta = -asin(Rm(3,1));
            phi   = atan2(Rm(3,2)/cos(theta), Rm(3,3)/cos(theta));
            psi   = atan2(Rm(2,1)/cos(theta), Rm(1,1)/cos(theta));
            R = [phi; theta; psi];

        case 'rotx'
            a = varargin{1};
            R = rotx_mat(a);

        case 'roty'
            a = varargin{1};
            R = roty_mat(a);

        case 'rotz'
            a = varargin{1};
            R = rotz_mat(a);

        case 'angle_axis'
            ax = varargin{1}; a = varargin{2};
            ax = ax / norm(ax);
            K = [0 -ax(3) ax(2); ax(3) 0 -ax(1); -ax(2) ax(1) 0];
            R = eye(3) + sin(a)*K + (1-cos(a))*K*K;

        otherwise
            error('Unknown rotation operation: %s', operation);
    end
end


function R = rotx_mat(a)
    R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function R = roty_mat(a)
    R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = rotz_mat(a)
    R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
end
