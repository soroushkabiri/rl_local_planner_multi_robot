    def attractive_part_apf(self):
        diff = self.current_way_point - self.rect_obj_position
        distance = np.linalg.norm(diff)
        if distance == 0:
            return np.zeros_like(diff)
        # Unit direction vector from leader to goal
        direction = diff / distance
        # Apply a constant magnitude force (capped by max_v_leader)
        F_att = self.k_att * direction
        F_att = 1*np.clip(F_att, 10*-self.max_v_leader , 10*self.max_v_leader)     

        return F_att
    
    def repulsive_part_apf(self):

        num_obs=self.position_obstacles.shape[0]
        # Initialize repulsive forces
        F_rep_p = np.array([np.zeros(2, dtype=np.float64)])
        F_rep_v = np.array([np.zeros(2, dtype=np.float64)])

        for i in range(num_obs): # in this loop we will be calculating cos_delta and threat level to use in repulsive potential
            # Calculate relative position and velocity
            p_relative = self.position_obstacles[i] - self.rect_obj_position
            velocity_relative = self.rect_obj_velocity - self.velocity_obstacles[i]
            # Calculate distance to obstacle
            dist_relative = np.linalg.norm(p_relative)
            # Calculate angle between relative position and relative velocity. this angle shows us that the obstacle and our object will be colliding or not. if cos_delta>0
            # then they will be colliding 
            if dist_relative == 0 or np.linalg.norm(velocity_relative) == 0:
                cos_delta = 0.0
            else:
                cos_delta = np.dot(p_relative, velocity_relative.T) / (dist_relative * np.linalg.norm(velocity_relative))
            delta = np.arccos(np.clip(cos_delta, -1, 1))
            # Calculate variable safety radius. the safety radius is the measure that act as on off switch in repulsive force calculation. r_safe=r_min_safe+r_var_safe
            if cos_delta > 0: # if cos_delta>0 -> the obstacle and our object are in dangerous situation -> r=r_safe+r_var_safe
                r_var_safe = (self.kv / (self.ka + self.max_omega_leader)) * np.linalg.norm(velocity_relative) * np.cos(delta) 
                # kv and ka are constants. max omega leader is in the denominator which is true. relative velocity and cos_delta have direct impact on r_var safe
                r_safe = self.r_safe_min + r_var_safe
            else:
                r_safe = self.r_safe_min
            # Calculate threat level
            if cos_delta > 0 and dist_relative > 0: # threat level is another factor that improve our dynamic collision avoidance
                TH_level = (1/dist_relative - 1/r_safe) * np.linalg.norm(velocity_relative) * np.cos(delta) # this factor has inverse relation with dis_relative and                                                                                            #direct relation with norm(velocity) and cos_delta
            else:
                TH_level = 0
            # repulsive force in potential field method has 2 parts. the first part happen because of the dist relative of obstacle and rectangular object.
            # Update forces if in danger zone
            if (dist_relative < r_safe) and (cos_delta > 0):
                # Position-based repulsive force
                F_rep_p += self.k_rep_p * TH_level * (-p_relative / dist_relative)
                # Velocity-based repulsive force. this is the second part of potential field repulsive force.
                if np.linalg.norm(velocity_relative) > 0:
                    F_rep_v += self.k_rep_v * TH_level * ((-p_relative/dist_relative) * (1/cos_delta) +velocity_relative/np.linalg.norm(velocity_relative))
        # Total repulsive force
        F_t = F_rep_p + F_rep_v
        return F_t

