function result = quaternion_utils(operation, varargin)
% QUATERNION_UTILS  Quaternion math utilities for rotation representation.
%
%   q = quaternion_utils('from_euler', phi, theta, psi)
%   [phi,theta,psi] = quaternion_utils('to_euler', q)
%   q = quaternion_utils('multiply', q1, q2)
%   q = quaternion_utils('conjugate', q)
%   q = quaternion_utils('normalize', q)
%   R = quaternion_utils('to_dcm', q)
%   v_rot = quaternion_utils('rotate_vector', q, v)
%
%   Quaternion convention: q = [w, x, y, z] (scalar-first)

    switch lower(operation)
        case 'from_euler'
            phi = varargin{1}; theta = varargin{2}; psi = varargin{3};
            cy = cos(psi/2); sy = sin(psi/2);
            cp = cos(theta/2); sp = sin(theta/2);
            cr = cos(phi/2); sr = sin(phi/2);

            result = [cr*cp*cy + sr*sp*sy;
                      sr*cp*cy - cr*sp*sy;
                      cr*sp*cy + sr*cp*sy;
                      cr*cp*sy - sr*sp*cy];

        case 'to_euler'
            q = varargin{1};
            w=q(1); x=q(2); y=q(3); z=q(4);

            % Roll
            sinr_cosp = 2*(w*x + y*z);
            cosr_cosp = 1 - 2*(x*x + y*y);
            phi = atan2(sinr_cosp, cosr_cosp);

            % Pitch
            sinp = 2*(w*y - z*x);
            if abs(sinp) >= 1
                theta = sign(sinp) * pi/2;
            else
                theta = asin(sinp);
            end

            % Yaw
            siny_cosp = 2*(w*z + x*y);
            cosy_cosp = 1 - 2*(y*y + z*z);
            psi = atan2(siny_cosp, cosy_cosp);

            result = [phi; theta; psi];

        case 'multiply'
            q1 = varargin{1}; q2 = varargin{2};
            w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
            w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);

            result = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
                      w1*x2 + x1*w2 + y1*z2 - z1*y2;
                      w1*y2 - x1*z2 + y1*w2 + z1*x2;
                      w1*z2 + x1*y2 - y1*x2 + z1*w2];

        case 'conjugate'
            q = varargin{1};
            result = [q(1); -q(2); -q(3); -q(4)];

        case 'normalize'
            q = varargin{1};
            result = q / norm(q);

        case 'to_dcm'
            q = varargin{1};
            q = q / norm(q);
            w=q(1); x=q(2); y=q(3); z=q(4);

            result = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
                      2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
                      2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];

        case 'rotate_vector'
            q = varargin{1}; v = varargin{2};
            q_v = [0; v(:)];
            q_conj = [q(1); -q(2:4)];
            q_rot = quat_mult(q, quat_mult(q_v, q_conj));
            result = q_rot(2:4);

        otherwise
            error('Unknown quaternion operation: %s', operation);
    end
end


function r = quat_mult(q1, q2)
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
    r = [w1*w2-x1*x2-y1*y2-z1*z2;
         w1*x2+x1*w2+y1*z2-z1*y2;
         w1*y2-x1*z2+y1*w2+z1*x2;
         w1*z2+x1*y2-y1*x2+z1*w2];
end
