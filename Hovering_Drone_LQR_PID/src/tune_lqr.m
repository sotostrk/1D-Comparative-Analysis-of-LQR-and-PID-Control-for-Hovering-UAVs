function [best_K, best_Q, best_R] = tune_lqr(m, g, target_st, target_os)
    A = [0 1; 0 0];
    B = [0 1/m]';
    C = [1 1];
    D = 0;

    found = false;

    for q1 = 100:20:300
        for q2 = 100:20:300
            for r = 1:1:10
                Q = diag([q1 q2]);
                R = r;
                K = lqr(A, B, Q, R);
                A_cl = A - B * K;
                sys_cl = ss(A_cl, B, C, D);
                S = stepinfo(sys_cl);

                if S.SettlingTime < target_st && S.Overshoot < target_os
                    fprintf(' Q=[%d %d], R=%d | ST=%.2fs, OS=%.2f%%\n', q1, q2, r, S.SettlingTime, S.Overshoot);
                    best_K = K; best_Q = Q; best_R = R;
                    found = true;
                    return;
                end
            end
            if found, break; end
        end
        if found, break; end
    end

    if ~found
        error(' No suitable Q/R found.');
    end
end