from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.mobility import MobilityHandler
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import VisualizationHandler, VisualizationConfiguration
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration
from simple_protocol import SimpleSensorProtocol, SimpleGroundStationProtocol, SimpleUAVProtocol, getRange


def main():
    # Configuring simulation
    config = SimulationConfiguration(
        duration=400
    )
    builder = SimulationBuilder(config)

    # Instantiating 1 sensors in fixed positions
    builder.add_node(SimpleSensorProtocol, (150, 0, 0))
    #builder.add_node(SimpleSensorProtocol, (0, 150, 0))
    #builder.add_node(SimpleSensorProtocol, (-150, 0, 0))
    #builder.add_node(SimpleSensorProtocol, (0, -150, 0))

    # Instantiating 4 UAVs at (0,0,0)
    builder.add_node(SimpleUAVProtocol, (0, 0, 0))
    '''
    builder.add_node(SimpleUAVProtocol, (0, 0, 0))
    builder.add_node(SimpleUAVProtocol, (0, 0, 0))
    builder.add_node(SimpleUAVProtocol, (0, 0, 0))
    '''

    # Instantiating ground station at (0,0,0)
    builder.add_node(SimpleGroundStationProtocol, (15, 15, 0))

    # Adding required handlers
    builder.add_handler(TimerHandler())
    builder.add_handler(CommunicationHandler(CommunicationMedium(
        transmission_range = getRange()
    )))
    builder.add_handler(MobilityHandler())
    builder.add_handler(VisualizationHandler(VisualizationConfiguration(
        x_range=(0, 500),
        y_range=(0, 500),
        z_range=(0, 500)
    )))

    # Building & starting
    simulation = builder.build()
    simulation.start_simulation()


if __name__ == "__main__":
    main()
