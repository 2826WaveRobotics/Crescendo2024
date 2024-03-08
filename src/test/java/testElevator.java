import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class testElevator {
    private Elevator elevator;

    @BeforeEach
    void setUp() {
        elevator = new Elevator();
    }

    @AfterEach
    void tearDown() {
        elevator = null;
    }

    @Test
    void testElevator() {
        assertEquals(0, elevator.getFloor());
    }

    @Test
    void testGoUp() {
        elevator.goUp();
        assertEquals(1, elevator.getFloor());
    }

    @Test
    void testGoDown() {
        elevator.goDown();
        assertEquals(-1, elevator.getFloor());
    }
}