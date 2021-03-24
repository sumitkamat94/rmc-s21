function traj(t::Float64)
    # compute the desired joint angle at time t
    return q_of_t
end


function control_PD!(τ, t, state)
    # Compute a value for τ

    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

function control_CTC!(τ, t, state)
    # Compute a value for τ

    # Saturate
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end
