def create_sgrid(args)
  args.state.sgrid_cell_size ||= args.state.avg_perception_radius
  args.state.sgrid_width ||= (Grid.w / args.state.sgrid_cell_size).ceil
  args.state.sgrid_height ||= (Grid.h / args.state.sgrid_cell_size).ceil
  args.state.spatial_grid ||= Array.new(args.state.sgrid_width) { Array.new(args.state.sgrid_height) { [] } }
end
  
def clear_sgrid(args)
  # Clear grid
  args.state.sgrid_width.times do |x|
    args.state.sgrid_height.times do |y|
      args.state.spatial_grid[x][y].clear
    end
  end
end

def update_sgrid(args)
  # Assign boids to grid cells
  args.state.boids.each do |boid|
    args.state.sgrid_x = (boid.x / args.state.sgrid_cell_size).floor.clamp(0, args.state.sgrid_width - 1)
    args.state.sgrid_y = (boid.y / args.state.sgrid_cell_size).floor.clamp(0, args.state.sgrid_height - 1)
    args.state.spatial_grid[args.state.sgrid_x][args.state.sgrid_y] << boid
  end
  
  # Update boids using neighboring cells
  args.state.boids.each do |boid|
    args.state.sgrid_x = (boid.x / args.state.sgrid_cell_size).floor.clamp(0, args.state.sgrid_width - 1)
    args.state.sgrid_y = (boid.y / args.state.sgrid_cell_size).floor.clamp(0, args.state.sgrid_height - 1)
    
    # Get neighbors from surrounding cells
    neighbors = []
    [args.state.sgrid_x-1, args.state.sgrid_x, args.state.sgrid_x+1].each do |nx|
      next if nx < 0 || nx >= args.state.sgrid_width
      [args.state.sgrid_y-1, args.state.sgrid_y, args.state.sgrid_y+1].each do |ny|
        next if ny < 0 || ny >= args.state.sgrid_height
        neighbors.concat(args.state.spatial_grid[nx][ny])
      end
    end
    
    # Limit number of neighbors processed
    if neighbors.length > 200
      neighbors = neighbors.sample(200)
    end
    
    # Update with limited neighbors
    boid.update(args, neighbors)
  end
end

def draw_sgrid(args)
  # Draw the spatial grid and highlight populated cells
  # Highlight populated cells with color intensity based on population
  args.state.sgrid_width.times do |x|
    args.state.sgrid_height.times do |y|
      cell = args.state.spatial_grid[x][y]
      if cell && !cell.empty?
        # Calculate color intensity based on number of boids in the cell
        # More boids = more intense color
        intensity = (cell.length / 10.0).clamp(0.2, 1.0)
        
        args.outputs[:rt_sgrid].primitives << {
          x: x * args.state.sgrid_cell_size,
          y: y * args.state.sgrid_cell_size,
          w: args.state.sgrid_cell_size,
          h: args.state.sgrid_cell_size,
          r: 0, 
          g: 200 * intensity, 
          b: 100 * intensity,
          a: 40 + (100 * intensity).to_i,
        }.solid!
        
        # Show boid count in each cell
        args.outputs[:rt_sgrid].primitives << {
          x: x * args.state.sgrid_cell_size + (args.state.sgrid_cell_size / 2),
          y: y * args.state.sgrid_cell_size + (args.state.sgrid_cell_size / 2),
          text: cell.length.to_s,
          alignment_enum: 1, # Center aligned
          vertical_alignment_enum: 1, # Middle aligned
          r: 255, g: 255, b: 255,
          a: 160,
          size_enum: -1 # Smaller text
        }.label!
      end
    end
  end
  
  # Add debug info about the grid
  
  # Display cell density stats
  if args.state.spatial_grid
    cell_counts = []
    args.state.spatial_grid.each do |row|
      row.each do |cell|
        cell_counts << cell.length if cell && !cell.empty?
      end
    end
    
    if !cell_counts.empty?
      max_count = cell_counts.max || 0
      avg_count = cell_counts.sum / cell_counts.length.to_f
      
      args.outputs.debug << "Grid stats:"
      args.outputs.debug << "    Size: #{args.state.sgrid_width}x#{args.state.sgrid_height} cells"
      args.outputs.debug << "    Max cell density: #{max_count} boids"
      args.outputs.debug << "    Avg cell density: #{avg_count.round(1)} boids"
      args.outputs.debug << "    Occupied cells: #{cell_counts.length} of #{args.state.sgrid_width * args.state.sgrid_height}"
    end
  end
end
