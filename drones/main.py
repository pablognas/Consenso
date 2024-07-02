from gradysim.simulator.handler.communication import CommunicationHandler, CommunicationMedium
from gradysim.simulator.handler.mobility import MobilityHandler
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import VisualizationHandler, VisualizationConfiguration
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration
from simple_protocol import SimpleSensorProtocol, SimpleGroundStationProtocol, SimpleUAVProtocol, getRange
from random import randint


def main():
    # Configuring simulation
    limiteMin = 0
    limiteMax = 500
    sensors = 10

    config = SimulationConfiguration(
        duration=400
    )
    builder = SimulationBuilder(config)

    # Instantiating 1 sensors in fixed positions
    for i in range(sensors):
        builder.add_node(SimpleSensorProtocol, (randint(limiteMin,limiteMax), randint(limiteMin,limiteMax), 0))
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
        x_range=(limiteMin, limiteMax),
        y_range=(limiteMin, limiteMax),
        z_range=(limiteMin, limiteMax/10)
    )))

    # Building & starting
    simulation = builder.build()
    simulation.start_simulation()


if __name__ == "__main__":
    main()
