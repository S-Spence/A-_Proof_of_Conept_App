import streamlit as st
import a_star_search
from typing import List

def print_world(world: List[List[str]]) -> None:
    new_world = "<br>".join("".join(row) for row in world)
    st.markdown(f'<pre>{new_world}</pre>', unsafe_allow_html=True)

def main():
    st.title("Find Shortest Path (A* Search)")

    # Game description
    st.markdown("The A* search algorithm finds the shortests path to a goal in the game world. Use the parameters in the sidebar to toggle the start position, end position, game world, actions, and costs.")
    st.markdown("Each symbol in the map has a different cost to travel there. The costs for the current run are displayed below. The game agent cannot pass through mountains '🗻' in the game world.")

    # Parameter selections
    st.sidebar.title("Parameters")
    world = a_star_search.full_world

    # Get world input
    st.sidebar.header("World Selection")
    world_selection = st.sidebar.selectbox("Choose game world", ["Large World", "Small World"], label_visibility="collapsed")

    # Set world
    if world_selection == "Small World":
        world = a_star_search.small_world

    # Create a form to collect input
    with st.sidebar.form("selection_form"):

         # Get cost input
        cost_labels = {'🌾': 'Plains', '🌲': 'Forest', '🪨': 'Hills', '🐊': 'Swamp'}
        st.header("Cost Selection")
        for symbol in a_star_search.COSTS:
            cost = st.number_input(f"{cost_labels[symbol]} {symbol} ", 0, 100, a_star_search.COSTS[symbol])
            a_star_search.COSTS[symbol] = cost

        # Get valid actions
        st.header("Actions Selection")
        moves_mapping = {'⏩': (0, 1), '⏪': (0, -1), '⏫': (-1, 0), '⏬': (1, 0)}
        moves = ['⏩', '⏪', '⏫', '⏬']
        valid_moves = st.multiselect("Select Actions:", moves, moves)
        MOVES = [moves_mapping[move] for move in valid_moves]

        # Get start and end position
        st.header("Position Selection")
        # Get starting row input
        start_row = st.number_input("Enter a starting row", 0, len(world)-1)
        # Get starting col input
        start_col = st.number_input("Enter a starting column", 0, len(world)-1)
        # Get goal row input
        goal_row = st.number_input("Enter a target row", 0, len(world[0])-1)
        # Get goal col input
        goal_col = st.number_input("Enter a target column", 0, len(world[0])-1)

        submitted = st.form_submit_button("Run A* Search")

    # Print costs
    st.header("Costs")
    for symbol, cost in a_star_search.COSTS.items():
        st.write(f"{cost_labels[symbol]} {symbol}: {cost}")

    # Print actions
    st.header("Actions")
    st.write(*valid_moves)

    # Print world
    if submitted:
        st.header("Solution")
        # Results output
        if world[start_col][start_row] == '🗻':
            st.write("Invalid starting position, cannot start on a mountain.")
            print_world(world)
        elif world[goal_col][goal_row] == '🗻':
            st.write("Invalid target position, mountains are unreachable.")
            print_world(world)
        else:
            solution = a_star_search.a_star_search(world, (start_col, start_row), (goal_col, goal_row), a_star_search.COSTS, MOVES, a_star_search.heuristic)
            path_cost, new_world = a_star_search.pretty_print_path(world, solution, (start_row, start_col), (goal_row, goal_col), a_star_search.COSTS)
            
            if len(solution) == 0:
                st.write("❌ NO SOLUTION")
                print_world(world)
            else:
                st.write(f"Path Cost: {path_cost}")
                st.write(f"Path Offsets: {solution[:-1]}")

                print_world(new_world)
    else:
        # Show the world with no path if the form is not submitted
        st.header("Game World")
        print_world(world)

if __name__ == "__main__":
    main()
