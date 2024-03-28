import { MainLayout } from './layouts/MainLayout';
import { RouterProvider } from 'react-router-dom';
import { router } from './router';
import { watchPositionHook } from './hooks/watchPositionHook';

export const App = () => {
	watchPositionHook();

	return (
		<>
			<MainLayout>
				<RouterProvider router={router} />
			</MainLayout>
		</>
	);
};
