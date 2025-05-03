# Boid algorithm implementation

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
        size_w_h: Numeric.rand(8.0..12.0),
        expired: false
      }
    else
      @dna = dna
    end
  end

  def update(boids)
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

    # expire birds after being alive for 6 seconds
    # TODO FIX
    # dna.expired = true if self.created_at_elapsed == 360 && args.state.gen_counter > 1

  end
end

def mutate(src_dna)
  # transform the inheritde dna data
  mutated = src_dna.transform_values do |val|
    val.is_a?(Numeric) ? val + Numeric.rand(-1.0..1.0) : val
  end

  # clamp values within acceptable ranges
  mutated.alignment_weight = mutated.alignment_weight.clamp(0.1, 3.0)
  mutated.cohesion_weight = mutated.cohesion_weight.clamp(0.1, 3.0)
  mutated.separation_weight = mutated.separation_weight.clamp(0.1, 3.0)
  mutated.perception_radius = mutated.perception_radius.clamp(20, 150)
  mutated.max_speed = mutated.max_speed.clamp(1.0, 6.0)
  mutated.size_w_h = mutated.size_w_h.clamp(6.0, 256.0)

  return mutated
end

def draw_boids(args)
  args.outputs.sprites << args.state.boids.map do |b|
    {
      x: b.x - (0.5 * b.dna.size),
      y: b.y - (0.5 * b.dna.size),
      w: b.dna.size_w_h,
      h: b.dna.size_w_h,
      angle: Math.atan2(b.vy, b.vx).to_degrees,
      path: 'sprites/isometric/red.png'
    }
  end
end

def tick(args)
  args.state.gen_counter ||= 1
  
  # initialize
  args.state.boids ||= Array.new(99) { Boid.new(rand(Grid.w), rand(Grid.h)) }

  # mutate every 5 seconds
  if  Kernel.tick_count.zmod?(300) && !Kernel.tick_count.zero?
    args.state.gen_counter += 1
    parent = args.state.boids.sample

    args.state.boids << Boid.new(parent.x, parent.y, mutate(parent.dna))

    # DEBUG
    p "#{args.state.boids.last} size: #{args.state.boids.last.dna.size_w_h}"
    # END DEBUG
  end

  # update
  args.state.boids.each { |boid| boid.update(args.state.boids) }

  # destroy dead boids
  # args.state.boids.reject! { |b| b.dna.expired == true }

  # draw
  draw_boids(args)

  # DEBUG
  # args.outputs.primitives << args.gtk.framerate_diagnostics_primitives
  args.outputs.debug << "#{args.gtk.current_framerate.to_sf}"
  args.outputs.debug << "generations: #{args.state.gen_counter}"
  args.outputs.debug << "living: #{args.state.boids.length}"
  # END DEBUG
end

