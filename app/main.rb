# Boid algorithm implementation

class Boid
  attr_accessor :x, :y, :vx, :vy

  def initialize(x, y)
    @x, @y = x, y
    # randomly pick a starting angle
    angle = rand * Math::PI * 2
    # use cos, sin to find the individual edges (aka velocities) for the x and y components of the angle (direction)
    @vx, @vy = Math.cos(angle) * 2, Math.sin(angle) * 2
  end

  def update(boids)
    # each boid will only be "conscious" of other boids within 50px
    perception_radius = 50
    # arrays for each part of the algoritm:
    # alignment (velocity), cohesion (direction), separation (social distancing)
    alignment = { x: 0, y: 0 }
    cohesion = { x: 0, y: 0 }
    separation = { x: 0, y: 0 }
    # keep a count of the total number of other boids perceived and processed
    total = 0

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
        separation.x += x - other.x
        separation.y += y - other.y
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
        # alignment.x = alignment.x / alignment_mag * 1.5
        # alignment.y = alignment.y / alignment_mag * 1.5
        alignment = Geometry.vec2_normalize(alignment)
        # scale to a strength of 1.5
        alignment.x *= 1.5
        alignment.y *= 1.5
      end

      # Cohesion
      # avg the group's position (center of mass)
      # subtract current boid's position to get the vector to the group
      # scale by 0.05 to adjust gently
      cohesion.x = ((cohesion.x / total) - x) * 0.05
      cohesion.y = ((cohesion.y / total) - y) * 0.05

      # Separation
      separation.x *= 0.25
      separation.y *= 0.25

      # Combine
      @vx += alignment.x + cohesion.x + separation.x
      @vy += alignment.y + cohesion.y + separation.y
    end

    # Limit speed
    speed = Math.hypot(vx, vy)
    max_speed = 4
    if speed > max_speed
      @vx = (@vx / speed) * max_speed
      @vy = (@vy / speed) * max_speed
    end

    # update the boid's position with the new vectors
    @x += @vx
    @y += @vy

    # Wrap around screen
    @x %= Grid.w
    @y %= Grid.h
  end
end

def draw_boids(args)
  args.outputs.sprites << args.state.boids.map do |b|
    {
      x: b.x - 5,
      y: b.y - 5,
      w: 16,
      h: 16,
      angle: Math.atan2(b.vy, b.vx).to_degrees,
      path: 'sprites/isometric/red.png'
    }
  end
end

def tick(args)
  # diagnostics
  # args.outputs.primitives << args.gtk.framerate_diagnostics_primitives
  args.outputs.labels << { x: Grid.left, y: Grid.top, text: "#{args.gtk.current_framerate.to_sf}" }
  
  # initialize
  args.state.boids ||= Array.new(100) { Boid.new(rand * Grid.w, rand * Grid.h) }
  # args.outputs.background_color = { r: 0, g: 0, b: 0, a: 100 }

  # update
  args.state.boids.each do |boid|
    boid.update(args.state.boids)
  end

  # draw
  draw_boids(args)
end

