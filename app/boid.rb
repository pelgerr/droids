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
        birth: Kernel.tick_count,
        alignment_weight: Numeric.rand(0.5..2.0),
        cohesion_weight: Numeric.rand(0.5..2.0),
        separation_weight: Numeric.rand(1.5..3.0),
        perception_radius: Numeric.rand(40..100),
        max_speed: Numeric.rand(2.0..5.0),
        size_w: 24,
        size_h: 18,
        vis_red: 255,
        vis_green: 255,
        vis_blue: 255
      }
    else
      @dna = dna
    end
  end

  def update(args, boids)
    # each boid will only be "conscious" of other boids within 50px
    perception_radius = dna.perception_radius
    alignment_weight = dna.alignment_weight
    cohesion_weight = dna.cohesion_weight
    separation_weight = dna.separation_weight
    

    # temp hashes for each part of the algoritm:
    # alignment (velocity), cohesion (direction), separation (social distancing)
    alignment = { x: 0.0, y: 0.0 }
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
      other_x = other.x
      other_y = other.y
      other_vx = other.vx
      other_vy = other.vy
      
      # find the distance (hypotenuse) between self and other_boid
      # skip if the current boid is self or outside of the perception radius
      next if other == self
      dist = Math.hypot(other_x - x, other_y - y)
      next if dist > perception_radius

      # Alignment - sum the x and y velocities of each perceived boid
      alignment.x += other_vx
      alignment.y += other_vy

      # Cohesion - sum the x and y positions of each perceived boid
      cohesion_x += other_x
      cohesion_y += other_y

      # Separation - try not to bump into other_boids
      if dist < 25
        # guard against divide by zero cases
        factor = 1.0 / (dist + 0.01)
        separation_x += (x - other_x) * factor
        separation_y += (y - other_y) * factor
      end

      total += 1
    end

    if total > 0
      # Alignment
      alignment.x /= total
      alignment.y /= total
      alignment_mag = Math.hypot(alignment.x, alignment.y)
      # normalize and scale the magnitude for smooth motion
      alignment = Geometry.vec2_normalize(alignment) if alignment_mag > 0

      # scale strength by alighnment weight
      alignment.x *= alignment_weight
      alignment.y *= alignment_weight

      # Cohesion
      # avg the group's position (center of mass)
      # subtract current boid's position to get the vector to the group
      # scale by 0.05 to adjust gently then apply the dna encoded cohesion weight
      cohesion_x = ((cohesion_x / total) - x) * 0.05
      cohesion_y = ((cohesion_y / total) - y) * 0.05
      cohesion_x *= cohesion_weight
      cohesion_y *= cohesion_weight

      # Separation
      separation_x *= separation_weight
      separation_y *= separation_weight

      # Combine steering forces
      steer_x += alignment.x + cohesion_x + separation_x
      steer_y += alignment.y + cohesion_y + separation_y
    end

    # Limit steering force
    steer_mag = Math.hypot(steer_x, steer_y)
    max_force = 0.5

    if steer_mag > max_force
      steer_x = (steer_x / steer_mag) * max_force
      steer_y = (steer_y / steer_mag) * max_force
    end

    # apply steering force to velocities
    @vx += steer_x
    @vy += steer_y

    # Limit speed
    speed = Math.hypot(@vx, @vy)
    min_speed = 0.5
    max_speed = dna.max_speed
    div_x_speed = @vx / speed
    div_y_speed = @vy / speed
    

    if speed > max_speed
      @vx = div_x_speed * max_speed
      @vy = div_y_speed * max_speed
    elsif speed < min_speed
      @vx = div_x_speed * min_speed
      @vy = div_y_speed * min_speed
    end

    # update the boid's position with the new vectors
    @x += @vx
    @y += @vy

    # Wrap around screen
    @x %= Grid.w
    @y %= Grid.h
  end
end

