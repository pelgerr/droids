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
        vis_blue: 255,
        expired: false
      }
    else
      @dna = dna
    end
  end

  def update(args, boids)
    # each boid will only be "conscious" of other boids within 50px
    perception_radius = dna.perception_radius

    # temp hashes for each part of the algoritm:
    # alignment (velocity), cohesion (direction), separation (social distancing)
    alignment = { x: 0, y: 0 }
    cohesion = { x: 0, y: 0 }
    separation = { x: 0, y: 0 }
    # keep a count of the total number of other boids perceived and processed
    total = 0
    # steer forces
    steer_x = 0.0
    steer_y = 0.0

    boids.each do |other|
      # find the distance (hypotenuse) between self and other boid
      # skip if the current boid is self or outside of the perception radius
      next if other == self
      dist = Math.hypot(other.x - x, other.y - y)
      next if dist > perception_radius

      # Alignment - sum the x and y velocities of each perceived boid
      alignment.x += other.vx
      alignment.y += other.vy

      # Cohesion - sum the x and y positions of each perceived boid
      cohesion.x += other.x
      cohesion.y += other.y

      # Separation - try not to bump into other boids
      if dist < 25
        # guard against divide by zero cases
        factor = 1.0 / (dist + 0.01)
        separation.x += (x - other.x) * factor
        separation.y += (y - other.y) * factor
      end

      total += 1
    end

    if total > 0
      # Alignment
      alignment.x /= total
      alignment.y /= total
      alignment_mag = Math.hypot(alignment.x, alignment.y)
      # normalize and scale the magnitude for smooth motion
      if alignment_mag > 0
        alignment = Geometry.vec2_normalize(alignment)
      end
      # scale strength by alighnment weight
      alignment.x *= dna.alignment_weight
      alignment.y *= dna.alignment_weight

      # Cohesion
      # avg the group's position (center of mass)
      # subtract current boid's position to get the vector to the group
      # scale by 0.05 to adjust gently then apply the dna encoded cohesion weight
      cohesion.x = ((cohesion.x / total) - x) * 0.05
      cohesion.y = ((cohesion.y / total) - y) * 0.05
      cohesion.x *= dna.cohesion_weight
      cohesion.y *= dna.cohesion_weight

      # Separation
      separation.x *= dna.separation_weight
      separation.y *= dna.separation_weight

      # Combine steering forces
      steer_x += alignment.x + cohesion.x + separation.x
      steer_y += alignment.y + cohesion.y + separation.y
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

    if speed > max_speed
      @vx = (@vx / speed) * max_speed
      @vy = (@vy / speed) * max_speed
    elsif speed < min_speed
      @vx = (@vx / speed) * min_speed
      @vy = (@vy / speed) * min_speed
    end

    # update the boid's position with the new vectors
    @x += @vx
    @y += @vy

    # Wrap around screen
    @x %= Grid.w
    @y %= Grid.h

    # TODO This is not a good lifespan simulation LOL
    args.state.boids.shift if args.state.boids.count > 200

    # DEBUG
    # if args.state.boids.count > 200
      # lucky = args.state.boids.shift
      # show_message("#{lucky} has expired")
    # end
    # END DEBUG

  end
end

