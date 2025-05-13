class Boid
  attr_accessor :x, :y, :vx, :vy, :dna

  def initialize(x, y, dna = nil)
    @x, @y = x, y
    # randomly pick a starting angle
    angle = rand * Math::PI * 2
    # use cos, sin to find the individual edges (aka velocities) for the x and y components of the angle (direction)
    @vx, @vy = Math.cos(angle) * 2, Math.sin(angle) * 2

    # DNA hash
    if dna.nil?
      @dna = {
        alignment_weight: Numeric.rand(0.5..2.0),
        cohesion_weight: Numeric.rand(0.5..2.0),
        separation_weight: Numeric.rand(1.5..3.0),
        perception_radius: Numeric.rand(40..100),
        max_speed: Numeric.rand(2.0..5.0),
        size_w: 22,
        size_h: 22,
        vis_red: 255,
        vis_green: 255,
        vis_blue: 255
      }
    else
      @dna = dna
    end
  end

  def update(args, boids)
    # each boid will only be "conscious" of other boids within the perception radius
    perception_radius = dna.perception_radius
    alignment_weight = dna.alignment_weight
    cohesion_weight = dna.cohesion_weight
    separation_weight = dna.separation_weight
    # Pre-calculate squared distances for more efficient comparisons
    perception_radius_squared = perception_radius * perception_radius
    separation_dist_squared = 25 * 25  

    # alignment (velocity), cohesion (direction), separation (social distancing)
    alignment_x = 0
    alignment_y = 0
    cohesion_x = 0
    cohesion_y = 0
    separation_x = 0
    separation_y = 0
    # keep a count of the total number of other boids perceived and processed
    total = 0

    # steer forces
    steer_x = 0.0
    steer_y = 0.0

    boids.each do |other|
      # skip if the current boid is self
      next if other == self

      # pre-calculations
      other_x = other.x
      other_y = other.y
      other_vx = other.vx
      other_vy = other.vy
      dx = other_x - x
      dy = other_y - y
      dist_squared = dx * dx + dy * dy
      
      # skip if the current boid is outside of the perception radius
      next if dist_squared > perception_radius_squared

      # Alignment - sum the x and y velocities of each perceived boid
      alignment_x += other_vx
      alignment_y += other_vy

      # Cohesion - sum the x and y positions of each perceived boid
      cohesion_x += other_x
      cohesion_y += other_y

      # Separation - try not to bump into other_boids
      if dist_squared < separation_dist_squared
        # find the distance between self and other_boid
        dist = Math.sqrt(dist_squared)
        # guard against divide by zero cases
        factor = 1.0 / (dist + 0.01)
        separation_x += (x - other_x) * factor
        separation_y += (y - other_y) * factor
      end

      # increase the total count of boids perceived
      total += 1
    end

    if total > 0
      # Alignment
      alignment_x /= total
      alignment_y /= total
      alignment_mag_squared = alignment_x * alignment_x + alignment_y * alignment_y
      # normalize and scale the magnitude for smooth motion
      if alignment_mag_squared > 0
        alignment_mag = Math.sqrt(alignment_mag_squared)
        alignment_x /= alignment_mag
        alignment_y /= alignment_mag
      end

      # WEIGHTS
      # scale by alighnment weight
      alignment_x *= alignment_weight
      alignment_y *= alignment_weight

      # Cohesion
      # avg the group's position (center of mass)
      # subtract current boid's position to get the vector to the group
      # scale by 0.05 to adjust gently then apply the cohesion weight
      cohesion_x = ((cohesion_x / total) - x) * 0.05
      cohesion_y = ((cohesion_y / total) - y) * 0.05
      cohesion_x *= cohesion_weight
      cohesion_y *= cohesion_weight

      # Separation
      separation_x *= separation_weight
      separation_y *= separation_weight

      # Combine steering forces
      steer_x += alignment_x + cohesion_x + separation_x
      steer_y += alignment_y + cohesion_y + separation_y
    end

    # Limit steering force
    steer_mag_squared = steer_x * steer_x + steer_y * steer_y
    max_force = 0.5
    max_force_squared = max_force * max_force

    if steer_mag_squared > max_force_squared
      steer_mag = Math.sqrt(steer_mag_squared)
      steer_x = (steer_x / steer_mag) * max_force
      steer_y = (steer_y / steer_mag) * max_force
    end

    # apply steering force to velocities
    @vx += steer_x
    @vy += steer_y

    # Limit speed
    # pre-calculations
    speed_squared = @vx * @vx + @vy * @vy
    min_speed = 0.5
    max_speed = dna.max_speed
    min_speed_squared = min_speed * min_speed
    max_speed_squared = max_speed * max_speed
    

    if speed_squared > max_speed_squared
      speed = Math.sqrt(speed_squared)
      @vx = (@vx / speed) * max_speed
      @vy = (@vy / speed) * max_speed
    elsif speed_squared < min_speed_squared
      speed = Math.sqrt(speed_squared)
      @vx = (@vx / speed) * min_speed
      @vy = (@vy / speed) * min_speed
    end

    # update the boid's position with the new vectors
    @x += @vx
    @y += @vy

    # Wrap around screen
    @x %= Grid.w
    @y %= Grid.h
  end
end


