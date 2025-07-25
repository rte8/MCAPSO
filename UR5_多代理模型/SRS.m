function x_new = SRS(X_bad,net,ps_input,ps_output)
    dim = 18;
    eta = 2;
    x_Repair = X_bad(1:18);
    y_c = X_bad(19) -1;
    Gradient = [];
    for j = 1:dim
        if rand > 0.5
            a = 1;
        else
            a = -1;
        end
        delta_x_Repair = x_Repair;
        delta_x_Repair(j) = delta_x_Repair(j) - eta;
        delta_c = TestRBF(delta_x_Repair,net,ps_input,ps_output);
        Gradient = [Gradient;(1/eta).*(delta_c - a*y_c)];
    end
    Gradient = Gradient';
    Grad_In = Gradient;
    x_new = x_Repair - (pinv(Grad_In)*y_c')';
end